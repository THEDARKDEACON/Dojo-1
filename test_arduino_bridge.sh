#!/bin/bash

# Test script for arduino_bridge

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash
source /home/robosync/Robot/Dojo/install/setup.bash

# Run the arduino_bridge_node with parameters
ros2 run arduino_bridge arduino_bridge_node --ros-args -p port:=/dev/ttyACM0 -p baud_rate:=115200
