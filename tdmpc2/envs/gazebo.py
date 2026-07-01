import os
import subprocess
import time
from os import path
import numpy as np
import rospy
import torch
import cv2
import gymnasium as gym

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from std_msgs.msg import Float32MultiArray, Float32, String
from collections import defaultdict

RESET_DELAY = 2.0
STEP_DELAY = 0.085
TOLERANCE = 0.001

def normalize_actions(action):
    min_linear, max_linear = 0.1, 1.0
    min_angular, max_angular = -0.9, 0.9

    # Linear velocity: map [-1, 1] to [0.1, 1.0]
    a_norm_linear = (action[0] + 1) / 2
    action[0] = a_norm_linear * (max_linear - min_linear) + min_linear
    # Angular velocity: map [-1, 1] to [-0.9, 0.9]
    a_norm_angular = (action[1] + 1) / 2
    action[1] = a_norm_angular * (max_angular - min_angular) + min_angular

    return action

def normalize_image(input_image, target_size):
    # Resize to target size
    img_resized = cv2.resize(input_image, (target_size, target_size), interpolation=cv2.INTER_LINEAR)
    # Convert to float32 and transpose from cv2 format (H, W, C) to pytorch's (C, H, W)
    img_processed = img_resized.astype(np.float32)
    return np.transpose(img_processed, (2, 0, 1))

class GazeboEnv(gym.Env):
    """Superclass for all Gazebo environments."""

    def __init__(self, cfg):
        super(GazeboEnv, self).__init__()
        
        #============ initial env setup
        self.cfg = cfg
        self.robot_ns = cfg.get("robot_ns", "robot")
        self.launchfile = str(cfg.launchfile)
        self.max_distance = cfg.get("max_distance")
        self.image_size = cfg.get("image_size", 64)
        self.obs_shape = cfg.get("obs_shape")  # e.g. [3,image_size,image_size]
        self.obs_type = cfg.get("obs", "rgb") 
        self.action_dim = cfg.get("action_dim")
        self.curriculum = cfg.get("curriculum_learning")
        self.vision_feature_size = 3*56*80
        self.in_eval = False
        self.last_lin_vel = 0.0
        self.last_ang_vel = 0.0
        self.filtered_action = torch.zeros(2, dtype=torch.float32, device=torch.get_default_device())
        if self.obs_type == "rgb":
            self.observation_space = gym.spaces.Box(
                low=0.0, high=255.0, shape=(3, self.image_size, self.image_size), dtype=np.float32
            )
        else:
            shape = (self.obs_shape,) if isinstance(self.obs_shape, int) else tuple(self.obs_shape)
            
            low_bounds_raw = np.array([0.1, -0.9])
            high_bounds_raw = np.array([1.0, 0.9])

            low_bounds_vision = np.full(self.vision_feature_size, -np.inf, dtype=np.float32)
            high_bounds_vision = np.full(self.vision_feature_size, np.inf, dtype=np.float32)

            final_low_bounds = np.concatenate([low_bounds_raw, low_bounds_vision])
            final_high_bounds = np.concatenate([high_bounds_raw, high_bounds_vision])

            self.observation_space = gym.spaces.Box(
                low=final_low_bounds, 
                high=final_high_bounds, 
                shape=shape, 
                dtype=np.float32
            )
        self.action_space = gym.spaces.Box(low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32)
        
        #============ robot moviments 
        self.pitch = self.roll = self.yaw = self.dis_error = 0.0
        self.current_x = self.max_x_position = 0.0
        self.vel_x = 1.0

        #============ algorithm variables
        self.heatmap_features = np.zeros(self.vision_feature_size, dtype=np.float32)
        self.last_heatmap_image = np.ones((224, 320, 3), dtype=np.uint8)
        self.state = np.zeros(self.obs_shape)
        self.reward = 0
        self._step = {'total': 0, 'ep': 0}
        self.done = False
        self.info = defaultdict(float, {'travel_dist': 0.0, 'collision_type': ''})

        #============ ROS setup
        # Integration points — see INTEGRATION.md for the full contract.
        # Required topics under /<robot_ns>/:
        #   SUB  ground_truth           [nav_msgs/Odometry]
        #   SUB  distance_error         [std_msgs/Float32MultiArray]
        #   SUB  vision/keypoint_heatmap        [std_msgs/Float32MultiArray]  (state obs)
        #   SUB  vision/keypoint_vis_argmax/compressed  [sensor_msgs/CompressedImage]  (rgb obs)
        #   PUB  cmd_vel                [geometry_msgs/Twist]
        ns = self.robot_ns
        subprocess.Popen(["roscore", "-p", "11311"])
        fullpath = self.launchfile if self.launchfile.startswith("/") else os.path.join(os.path.dirname(__file__), "", self.launchfile)
        if not path.exists(fullpath):
            raise IOError("File " + fullpath + " does not exist")

        rospy.init_node("gym", anonymous=True)
        subprocess.Popen(["roslaunch", "-p", "11311", fullpath])
        time.sleep(8)

        # services
        self.unpause = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        self.pause = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self.reset_proxy = rospy.ServiceProxy("/gazebo/reset_world", Empty)

        # pubs
        self.terra_vel_pub = rospy.Publisher(f"/{ns}/cmd_vel", Twist, queue_size=10)
        self.collision_pub = rospy.Publisher(f"{ns}/collision", String, queue_size=1)
        self.collision_max_x_pub = rospy.Publisher(f"{ns}/max_x_position", Float32, queue_size=1)

        # subs
        self.odom_sub = rospy.Subscriber(f"/{ns}/ground_truth", Odometry, self.odom_callback, queue_size=15)
        self.dis_error_sub = rospy.Subscriber(f"/{ns}/distance_error", Float32MultiArray, self.d_error_callback, queue_size=15)
        self.vis_heatmap_sub = rospy.Subscriber(f'/{ns}/vision/keypoint_vis_argmax/compressed', CompressedImage, self.image_rect_color_callback, queue_size=15)
        self.head_sub = rospy.Subscriber(f'/{ns}/vision/keypoint_heatmap', Float32MultiArray, self.head_callback, queue_size=1)

    def head_callback(self, msg):
        self.heatmap_features = np.array(msg.data, dtype=np.float32)

    def d_error_callback(self, dis_error):
        self.dis_error = float(dis_error.data[0])

    def image_rect_color_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)  
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.last_heatmap_image = image_rgb

    def odom_callback(self, od_data):
        self.vel_x = od_data.twist.twist.linear.x
        self.pitch = od_data.twist.twist.angular.y
        self.roll = od_data.twist.twist.angular.x
        self.yaw = od_data.twist.twist.angular.z
        self.current_x = od_data.pose.pose.position.x

        if self.max_x_position == 0.0 or self.current_x > self.max_x_position:
            self.max_x_position = self.current_x

        if self.current_x <= TOLERANCE and self.max_x_position > TOLERANCE:
            self.max_x_position = 0.0

    def rand_act(self):
        return torch.from_numpy(self.action_space.sample().astype(np.float32))

    def step(self, action):
        # update step counter, note that total should only be updated in training stage
        self._step.update({
            'ep': self._step['ep'] + 1,
            'total': self._step['total'] + (0 if self.in_eval else 1)
        })

        #============ action update
        robot_action = normalize_actions(action.clone())
        self.filtered_action = self.filtered_action.to(robot_action.device)

        alpha = 0.3
        self.filtered_action = alpha * robot_action + (1 - alpha) * self.filtered_action
        self.vel_cmd = Twist()
        self.vel_cmd.linear.x = self.filtered_action[0].item()
        self.vel_cmd.angular.z = self.filtered_action[1].item()
        self.terra_vel_pub.publish(self.vel_cmd)

        #============ simulation run
        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except rospy.ServiceException as e:
            print("/gazebo/unpause_physics service call failed")

        time.sleep(STEP_DELAY)

        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except rospy.ServiceException as e:
            print("/gazebo/pause_physics service call failed")

        #============ state update
        # Use RGB image directly: resize+normalize to (3,image_size,image_size)
        if self.obs_type == 'rgb':
            self.state = normalize_image(self.last_heatmap_image, self.image_size)
        else:
            robot_action_np = self.filtered_action.cpu().numpy()
            raw_state = np.array([robot_action_np[0], robot_action_np[1]])
            self.state = np.concatenate([raw_state, self.heatmap_features])

        # ============ reward update
        collision = self.observe_collision()
        self.reward = self.get_reward(collision['response'], self.filtered_action[0].item(), self.filtered_action[1].item())
        self.last_lin_vel = self.filtered_action[0].item()
        self.last_ang_vel = self.filtered_action[1].item()
        self.done = collision['response'] or self.current_x > self.max_distance
        self.info['travel_dist'] = self.current_x
        self.info['collision_type'] = collision['type']

        # ============ collision case
        if self.done:
            self.collision_pub.publish(collision['type'])
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = 0
            self.current_x = 0
            self.collision_max_x_pub.publish(round(self.max_x_position, 2))
            self.terra_vel_pub.publish(self.vel_cmd)

        # ============ return formatting
        obs = torch.tensor(self.state.flatten(), dtype=torch.float32, device=torch.get_default_device())
        reward = torch.tensor(self.reward) if not isinstance(self.reward, torch.Tensor) else self.reward

        return obs, reward, self.done, self.info

    def reset(self):

        self._step.update({'ep': 0})
        self.last_lin_vel = 0.0
        self.last_ang_vel = 0.0
        self.filtered_action.zero_()

        #============ simulation run
        rospy.wait_for_service("/gazebo/reset_world")
        try:
            self.reset_proxy()
        except rospy.ServiceException as e:
            print("/gazebo/reset_simulation service call failed")

        rospy.wait_for_service("/gazebo/unpause_physics")
        try:
            self.unpause()
        except rospy.ServiceException as e:
            print("/gazebo/unpause_physics service call failed")

        time.sleep(RESET_DELAY)

        rospy.wait_for_service("/gazebo/pause_physics")
        try:
            self.pause()
        except rospy.ServiceException as e:
            print("/gazebo/pause_physics service call failed")

        # Use RGB image directly: resize+normalize to (3,image_size,image_size)
        if self.obs_type == "rgb":
            self.state = normalize_image(self.last_heatmap_image, self.image_size)
        else:
            raw_state = np.array([self.vel_x, self.yaw])
            self.state = np.concatenate([raw_state, self.heatmap_features])

        # ============ output formatting
        obs = torch.tensor(self.state.flatten(), dtype=torch.float32, device=torch.get_default_device())
        return obs

    def observe_collision(self):
        """
        Detects collision conditions based on current robot state
        
        Uses internal state variables: self.dis_error, self.vel_x, self.vel_cmd,
        self.pitch, self.roll, self._step, self.cfg.steps, self.curriculum
        
        Returns: Dict with 'response' (bool) and 'type' (str or None)
        """
        # Allow a few steps at the start of each episode before checking collisions
        if 0 < self._step['ep'] < 10:
            return {'response': False, 'type': None}

        distance = 1.9 if self.curriculum and self._step['total'] < self.cfg.steps*0.1 else 0.62

        if (abs(self.vel_x) < 0.15 and abs(self.vel_cmd.linear.x) > 0.25):
            return {'response': True, 'type': 'stuck'}
            
        if abs(self.dis_error) > distance:
            if self.dis_error < 0:
                return {'response': True, 'type': 'distance left'}
            else:
                return {'response': True, 'type': 'distance right'}
                
        if abs(self.pitch) > 0.01 or abs(self.roll) > 0.01:
            return {'response': True, 'type': 'acrobatic'}
            
        return {'response': False, 'type': None}

    def get_reward(self, collision, vel_lin, vel_ang):
        ''' Logistic softplus reward function
        dense_rw: some delta x-axis variable, but with a non-linear transformation to make it more interesting
        discounts_rw: what we are trying to minimize
        '''
        dense_rw = np.exp(- ((vel_lin-1)**2) / 0.1)
        discounts_rw = 1.0*(abs(vel_lin - self.last_lin_vel)) + 1.0*(abs(vel_ang - self.last_ang_vel)) + 2*(vel_ang**2) + 50 * collision  

        total_reward = (dense_rw) / (1 + np.exp(0.6 * discounts_rw)) # logistic softplus  
        
        return total_reward

def make_gazebo_env(cfg):
    return GazeboEnv(cfg)
