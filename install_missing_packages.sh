#!/bin/bash

# Install missing ROS2 packages for simulation

echo "🔧 Installing missing ROS2 packages..."

# Update package list
apt-get update

# Install missing ROS2 packages
apt-get install -y \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    ros-humble-nav2-map-server \
    ros-humble-teleop-twist-keyboard

echo "✅ Missing packages installed successfully!"
echo ""
echo "Now you can run the full simulation:"
echo "  ./run_full_simulation.sh"