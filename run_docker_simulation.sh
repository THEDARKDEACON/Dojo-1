#!/bin/bash

# Dojo Robot Docker Simulation Launcher
# Optimized for Docker containers with optional GUI support

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}🐳 Starting Dojo Robot Docker Simulation${NC}"
echo ""

# Check if we're in Docker
if [ -f /.dockerenv ]; then
    echo -e "${GREEN}✅ Running in Docker container${NC}"
else
    echo -e "${YELLOW}⚠️  Not in Docker container - consider using run_full_simulation.sh instead${NC}"
fi

# Check if workspace is built
if [ ! -f "install/setup.bash" ]; then
    echo -e "${YELLOW}⚠️  Workspace not built. Building now...${NC}"
    ./build_ros2.sh
fi

# Source the workspace
echo -e "${BLUE}📦 Sourcing workspace...${NC}"
source install/setup.bash

# Parse command line arguments
WORLD=${1:-"dojo_world.world"}
GUI=${2:-"false"}
RVIZ=${3:-"false"}
SLAM=${4:-"true"}

# Check for X11 forwarding if GUI requested
if [ "$GUI" = "true" ] || [ "$RVIZ" = "true" ]; then
    if [ -z "$DISPLAY" ]; then
        echo -e "${RED}❌ GUI requested but no DISPLAY variable set${NC}"
        echo -e "${YELLOW}💡 To enable GUI, run Docker with: -e DISPLAY=\$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix${NC}"
        echo -e "${YELLOW}   Falling back to headless mode...${NC}"
        GUI="false"
        RVIZ="false"
    else
        echo -e "${GREEN}✅ X11 forwarding detected${NC}"
    fi
fi

echo -e "${GREEN}🎮 Docker Simulation Configuration:${NC}"
echo "  World: $WORLD"
echo "  Gazebo GUI: $GUI"
echo "  RViz: $RVIZ"
echo "  SLAM Mapping: $SLAM"
echo "  Headless Mode: $([ "$GUI" = "false" ] && echo "true" || echo "false")"
echo ""

echo -e "${BLUE}🌍 Launching Docker-optimized simulation...${NC}"
echo ""

if [ "$GUI" = "false" ]; then
    echo -e "${YELLOW}📊 Monitoring Topics (in separate terminals):${NC}"
    echo "  ros2 topic echo /scan          # LiDAR data"
    echo "  ros2 topic echo /odom          # Odometry"
    echo "  ros2 topic echo /map           # SLAM map"
    echo "  ros2 topic list               # All topics"
    echo ""
    echo -e "${YELLOW}🎮 Control Robot:${NC}"
    echo "  ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'"
    echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=cmd_vel_manual"
    echo ""
fi

echo -e "${YELLOW}Press Ctrl+C to stop simulation${NC}"
echo ""

# Launch the Docker-optimized simulation
ros2 launch robot_gazebo docker_simulation.launch.py \
    world:=$WORLD \
    gui:=$GUI \
    rviz:=$RVIZ \
    slam:=$SLAM \
    headless:=$([ "$GUI" = "false" ] && echo "true" || echo "false")