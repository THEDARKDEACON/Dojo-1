#!/bin/bash

# Simulation Monitor Script
# Shows real-time data from the simulation

set -e

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}📊 Dojo Robot Simulation Monitor${NC}"
echo ""

# Source workspace
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/humble/setup.bash
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    fi
fi

# Function to check if topic exists
check_topic() {
    local topic=$1
    if ros2 topic list | grep -q "^$topic$"; then
        echo -e "${GREEN}✅${NC}"
    else
        echo -e "${RED}❌${NC}"
    fi
}

# Function to get topic rate
get_rate() {
    local topic=$1
    timeout 3s ros2 topic hz "$topic" 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "N/A"
}

while true; do
    clear
    echo -e "${BLUE}📊 Dojo Robot Simulation Status${NC}"
    echo "$(date)"
    echo ""
    
    echo -e "${YELLOW}🤖 Robot Topics:${NC}"
    printf "%-25s %-10s %-15s\n" "Topic" "Status" "Rate (Hz)"
    printf "%-25s %-10s %-15s\n" "-----" "------" "---------"
    printf "%-25s %-10s %-15s\n" "/cmd_vel" "$(check_topic /cmd_vel)" "$(get_rate /cmd_vel)"
    printf "%-25s %-10s %-15s\n" "/odom" "$(check_topic /odom)" "$(get_rate /odom)"
    printf "%-25s %-10s %-15s\n" "/scan" "$(check_topic /scan)" "$(get_rate /scan)"
    printf "%-25s %-10s %-15s\n" "/image_raw" "$(check_topic /image_raw)" "$(get_rate /image_raw)"
    printf "%-25s %-10s %-15s\n" "/map" "$(check_topic /map)" "$(get_rate /map)"
    printf "%-25s %-10s %-15s\n" "/joint_states" "$(check_topic /joint_states)" "$(get_rate /joint_states)"
    echo ""
    
    echo -e "${YELLOW}🎮 Control Topics:${NC}"
    printf "%-25s %-10s %-15s\n" "/cmd_vel_manual" "$(check_topic /cmd_vel_manual)" "$(get_rate /cmd_vel_manual)"
    printf "%-25s %-10s %-15s\n" "/cmd_vel_filtered" "$(check_topic /cmd_vel_filtered)" "$(get_rate /cmd_vel_filtered)"
    echo ""
    
    echo -e "${YELLOW}🗺️  SLAM Topics:${NC}"
    printf "%-25s %-10s %-15s\n" "/map" "$(check_topic /map)" "$(get_rate /map)"
    printf "%-25s %-10s %-15s\n" "/map_metadata" "$(check_topic /map_metadata)" "$(get_rate /map_metadata)"
    echo ""
    
    echo -e "${YELLOW}📡 Active Nodes:${NC}"
    ros2 node list | head -10
    echo ""
    
    echo -e "${YELLOW}💡 Quick Commands:${NC}"
    echo "  Drive forward:  ros2 topic pub /cmd_vel_manual geometry_msgs/Twist '{linear: {x: 0.2}}' --once"
    echo "  Turn left:      ros2 topic pub /cmd_vel_manual geometry_msgs/Twist '{angular: {z: 0.5}}' --once"
    echo "  Stop:           ros2 topic pub /cmd_vel_manual geometry_msgs/Twist '{}' --once"
    echo "  View LiDAR:     ros2 topic echo /scan --once"
    echo "  View map info:  ros2 topic echo /map_metadata --once"
    echo ""
    echo -e "${BLUE}Press Ctrl+C to exit monitor${NC}"
    
    sleep 2
done