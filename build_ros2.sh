#!/bin/bash

# Source ROS 2 setup file
source /opt/ros/humble/setup.bash

# Print environment variables for debugging
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_PYTHON_VERSION: $ROS_PYTHON_VERSION"

# Build the workspace
cd /home/Dojo/Dojo
colcon build --symlink-install --packages-select arduino_bridge ros2arduino_bridge

# Source the workspace
source install/setup.bash

# List the installed packages
ros2 pkg list | grep -E "arduino_bridge|ros2arduino_bridge"
