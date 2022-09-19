#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/humble/setup.bash" --
cd ~/ros2_ws
colcon build
source "install/setup.bash" --
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
export ROS_DOMAIN_ID=30
exec "$@"
