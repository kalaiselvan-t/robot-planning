#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/humble/setup.bash" --
cd ~/ros2_ws
colcon build
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=3" >> ~/.bashrc
exec "$@"
