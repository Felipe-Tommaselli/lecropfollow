#!/usr/bin/env python3
"""
Ground-truth row-following error publisher.

Reads the robot's ground-truth pose from Gazebo and computes the lateral
distance error and heading error relative to the crop row grid.

Topics published (under /<robot_ns>/):
  distance_error  [std_msgs/Float32MultiArray]  lateral offset from row centre (m)
  heading_error   [std_msgs/Float32MultiArray]  heading deviation (rad)

ROS params:
  /robot_ns  (default: "robot")  — must match the robot_ns in config.yaml
"""
import sys
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from scipy.spatial.transform import Rotation as R


class RowfollowGt:
    def __init__(self, lane_width, robot_ns):
        self.lane_width = lane_width
        self.row_num = 0

        self.gt_sub = rospy.Subscriber(
            f'/{robot_ns}/ground_truth', Odometry,
            callback=self.gt_callback, queue_size=1
        )
        self.heading_pub = rospy.Publisher(
            f'/{robot_ns}/heading_error', Float32MultiArray, queue_size=1
        )
        self.distance_pub = rospy.Publisher(
            f'/{robot_ns}/distance_error', Float32MultiArray, queue_size=1
        )

    def gt_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.row_num_update(pos)
        yaw, roll, pitch = R.from_quat([ori.x, ori.y, ori.z, ori.w]).as_euler('zxy', degrees=False)

        if self.row_num % 2 == 0:
            heading = -yaw
            dl_cg = (-pos.y - (self.row_num * self.lane_width))
            dr_cg = self.lane_width - dl_cg
        else:
            heading = np.pi - yaw
            dr_cg = (-pos.y - (self.row_num * self.lane_width))
            dl_cg = self.lane_width - dr_cg

        l1 = np.array([-dl_cg, 0, -pos.z])
        l2 = np.array([-dl_cg, 0.23222, -pos.z])
        r2 = np.array([dr_cg, 0.23222, -pos.z])

        van_lines_rot = R.from_euler('zyx', [-yaw, -pitch, -roll], degrees=False).as_matrix()
        l2 = van_lines_rot @ l2
        r2 = van_lines_rot @ r2

        distance_error = -l2[0] - r2[0]
        heading = (heading + np.pi) % (2 * np.pi) - np.pi

        self.heading_pub.publish(Float32MultiArray(data=[heading]))
        self.distance_pub.publish(Float32MultiArray(data=[distance_error]))

    def row_num_update(self, pos):
        self.row_num = 0


def main():
    rospy.init_node('rowfollow_gt_node')
    robot_ns = rospy.get_param('/robot_ns', 'robot')
    lane_width = rospy.get_param('~lane_width', 0.76)
    RowfollowGt(lane_width, robot_ns)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    sys.exit(main() or 0)
