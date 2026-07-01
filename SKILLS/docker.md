key files: docker/ROS_noetic.dockerfile, docker/environment.yaml, docker/config/entrypoint.sh

build: bash docker/scripts/build.sh
run:   bash docker/scripts/run.sh

base image: osrf/ros:noetic-desktop-full
user created in image: tommaselli (UID 1000), passwordless sudo

apt packages installed (docker/ROS_noetic.dockerfile:29-51):
ros-noetic-gazebo-dev        Gazebo plugin headers + gazebo_ros pkg
ros-noetic-hector-gazebo-plugins  required by terra_description C++ headers (sensor_model.h etc.)
ros-noetic-cv-bridge         Python↔OpenCV bridge for ROS images
build-essential cmake pkg-config git curl wget python3 python3-pip python3-catkin-tools ffmpeg libgl1-mesa-*

conda env (docker/environment.yaml): installs into base env via `conda env update -n base`
PyTorch: installed via pip with --extra-index-url https://download.pytorch.org/whl/cu124 (CUDA 12.4)
key pip packages: gymnasium, hydra-core, wandb, tensordict, termcolor, colorful, scipy, opencv-python, rospy (via system ROS), torch/torchvision

env conflicts (important):
LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu:/opt/ros/noetic/lib  (pinned, no conda libs)
PYTHONPATH=""  (intentional — conda must not override ROS Python path)
libffi7 reinstalled explicitly to resolve cv_bridge/conda libffi conflict

entrypoint (docker/config/entrypoint.sh):
source /opt/ros/noetic/setup.bash
source /workspace/devel/setup.bash  (if built; skipped if not yet built)
exec "$@"

GAZEBO_PLUGIN_PATH: /usr/lib/x86_64-linux-gnu/gazebo-11/plugins (set in Dockerfile ENV)

catkin workspace: /ros_ws (outside this repo). repo cloned into /ros_ws/lecropfollow.
build command inside container: cd /ros_ws && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 && source devel/setup.bash
no C++ targets in the new src/ packages — catkin_make is fast (only processes CMakeLists boilerplate + Python install).
