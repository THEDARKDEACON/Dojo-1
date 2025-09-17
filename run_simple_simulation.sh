#!/bin/bash

# Simple Dojo Robot Simulation Launcher
# Basic simulation without external dependencies

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚀 Starting Simple Dojo Robot Simulation${NC}"
echo ""

# Check if workspace is built
if [ ! -f "install/setup.bash" ]; then
    echo -e "${YELLOW}⚠️  Workspace not built. Building now...${NC}"
    ./build_ros2.sh
fi

# Source the workspace
echo -e "${BLUE}📦 Sourcing workspace...${NC}"
source install/setup.bash

# Launch options
WORLD=${1:-"dojo_world.world"}
GUI=${2:-"true"}
RVIZ=${3:-"true"}

echo -e "${GREEN}🎮 Simple Simulation Configuration:${NC}"
echo "  World: $WORLD"
echo "  Gazebo GUI: $GUI"
echo "  RViz: $RVIZ"
echo ""

echo -e "${BLUE}🌍 Launching simple simulation...${NC}"
echo ""
echo -e "${YELLOW}Features:${NC}"
echo "  ✅ Robot motion and control"
echo "  ✅ LiDAR sensor data"
echo "  ✅ Camera feed"
echo "  ✅ Teleop control (xterm window)"
echo "  ✅ RViz visualization"
echo ""
echo -e "${YELLOW}Controls:${NC}"
echo "  - Use the teleop terminal window to drive the robot"
echo "  - LiDAR data appears as red points in RViz"
echo "  - Camera feed is shown in RViz Image panel"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop simulation${NC}"
echo ""

# Launch the simple simulation
ros2 launch robot_gazebo simple_simulation.launch.py \
    world:=$WORLD \
    gui:=$GUI \
    rviz:=$RVIZ \
    use_teleop:=true