#!/bin/bash

# Test script for teleop_twist_keyboard with arduino_bridge

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash
source /home/robosync/Robot/Dojo/install/setup.bash

# Launch the test_arduino.launch.py file
ros2 launch arduino_bridge test_arduino.launch.py
