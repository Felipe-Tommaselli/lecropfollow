full contract: SKILLS/integration.md
key code: tdmpc2/envs/gazebo.py:100-135 (all pub/sub setup)

robot_ns: set in tdmpc2/config.yaml and passed as arg to terra_gazebo.launch; all topics prefixed /<robot_ns>/

subscribed by GazeboEnv (tdmpc2 reads):
/<ns>/ground_truth                         nav_msgs/Odometry      robot pose+velocity (p3d plugin)
/<ns>/distance_error                       std_msgs/Float32MultiArray  data[0]=lateral offset from row (m)
/<ns>/vision/keypoint_heatmap             std_msgs/Float32MultiArray  13440 floats for state obs
/<ns>/vision/keypoint_vis_argmax/compressed  sensor_msgs/CompressedImage  for rgb obs mode

published by GazeboEnv (tdmpc2 writes):
/<ns>/cmd_vel          geometry_msgs/Twist  velocity command (linear.x, angular.z) — NOT TwistStamped
/<ns>/collision        std_msgs/String      episode-end collision type string
/<ns>/max_x_position   std_msgs/Float32     max forward travel on episode end

Gazebo services (std_srvs/Empty, always at these paths):
/gazebo/unpause_physics   called before each step action
/gazebo/pause_physics     called after each step action + after reset settle
/gazebo/reset_world       called on each reset()

src/ packages:
robot_description  urdf/robot.urdf.xacro — diff-drive + p3d + camera, no C++ build needed
terra_gazebo       launch/terra_gazebo.launch — sets GAZEBO_MODEL_PATH, spawns robot, starts utility nodes
terra_worlds       worlds/farm.world + models/ (90 Gazebo models, crop rows + terrain)
terra_utils        scripts/rowfollow_gt.py (GT distance/heading), scripts/perception_stub.py (zero heatmap)

terra_gazebo.launch args:
robot_ns (default: robot)   must match config.yaml robot_ns
world (default: farm.world) relative to terra_worlds/worlds/
gui (default: false)        set true to open Gazebo GUI
robot_model                 path to xacro, default $(find robot_description)/urdf/robot.urdf.xacro

rowfollow_gt.py reads /robot_ns ROS param (set by launch as global param /robot_ns).
perception_stub.py reads same; publishes at 10 Hz; requires cv2 for blank jpeg encoding.

to replace robot: swap src/robot_description/urdf/robot.urdf.xacro, ensure it accepts robot_ns xacro arg, publishes ground_truth Odometry and subscribes cmd_vel Twist under same namespace.
to replace perception: swap src/terra_utils/scripts/perception_stub.py with node that publishes real 13440-float heatmap on /<ns>/vision/keypoint_heatmap.
