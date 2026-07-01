cmake_minimum_required(VERSION 3.0.2)
project(terra_utils)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs nav_msgs)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/rowfollow_gt.py
  scripts/perception_stub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
