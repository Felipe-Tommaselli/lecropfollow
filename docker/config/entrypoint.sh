#!/bin/bash

set -e
source /opt/ros/noetic/setup.bash
# Source the catkin workspace if it has already been built
[ -f /workspace/devel/setup.bash ] && source /workspace/devel/setup.bash
exec "$@"
