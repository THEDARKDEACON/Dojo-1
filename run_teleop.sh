#!/bin/bash

# Simple Teleop Script for Docker
# Run this in a separate terminal to control the robot

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}🎮 Dojo Robot Teleop Control${NC}"
echo ""

# Check if workspace is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}📦 Sourcing ROS workspace...${NC}"
    source /opt/ros/humble/setup.bash
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    fi
fi

echo -e "${GREEN}🚀 Starting keyboard teleop...${NC}"
echo ""
echo -e "${YELLOW}Controls:${NC}"
echo "   u    i    o"
echo "   j    k    l" 
echo "   m    ,    ."
echo ""
echo "q/z : increase/decrease max speeds by 10%"
echo "w/x : increase/decrease only linear speed by 10%"
echo "e/c : increase/decrease only angular speed by 10%"
echo "space key, k : force stop"
echo "anything else : stop smoothly"
echo ""
echo "CTRL-C to quit"
echo ""

# Run teleop with remapping to control manager
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=cmd_vel_manual