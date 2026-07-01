#!/usr/bin/env python3
"""
Perception stub — placeholder for the crop-row keypoint perception model.

Publishes zero-filled heatmap features so the TD-MPC2 pipeline can run
end-to-end without a real perception model. The agent will train on
meaningless vision features until this stub is replaced.

Topics published (under /<robot_ns>/):
  vision/keypoint_heatmap         [std_msgs/Float32MultiArray]
      Shape: (3 * 56 * 80,) = 13440 floats — same shape the RL agent expects.
  vision/keypoint_vis_argmax/compressed  [sensor_msgs/CompressedImage]
      Blank 224×320 RGB image — satisfies the rgb obs-mode subscriber.

To replace with real perception:
  1. Implement a node that subscribes to the robot's camera topic and runs
     your keypoint model.
  2. Publish the heatmap output on the same topics above.
  3. Swap this node for yours in terra_gazebo/launch/terra_gazebo.launch.

ROS params:
  /robot_ns  (default: "robot")
"""
import sys
import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import CompressedImage

HEATMAP_SIZE = 3 * 56 * 80   # must match obs_shape in config.yaml minus 2
IMAGE_H, IMAGE_W = 224, 320


def main():
    rospy.init_node('perception_stub_node')
    robot_ns = rospy.get_param('/robot_ns', 'robot')
    rate_hz = rospy.get_param('~rate', 10.0)

    heatmap_pub = rospy.Publisher(
        f'/{robot_ns}/vision/keypoint_heatmap',
        Float32MultiArray, queue_size=1
    )
    image_pub = rospy.Publisher(
        f'/{robot_ns}/vision/keypoint_vis_argmax/compressed',
        CompressedImage, queue_size=1
    )

    zero_heatmap = Float32MultiArray(data=[0.0] * HEATMAP_SIZE)

    blank_img = CompressedImage()
    blank_img.format = 'jpeg'
    blank_pixels = np.zeros((IMAGE_H, IMAGE_W, 3), dtype=np.uint8)
    import cv2
    _, buf = cv2.imencode('.jpg', blank_pixels)
    blank_img.data = buf.tobytes()

    rate = rospy.Rate(rate_hz)
    rospy.loginfo(f'[perception_stub] publishing on /{robot_ns}/vision/* at {rate_hz} Hz')
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        zero_heatmap.layout.data_offset = 0
        heatmap_pub.publish(zero_heatmap)
        blank_img.header.stamp = now
        image_pub.publish(blank_img)
        rate.sleep()


if __name__ == '__main__':
    sys.exit(main() or 0)
