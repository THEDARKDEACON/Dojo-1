#!/bin/bash

# Dojo Robot Full Simulation Launcher
# Launches complete simulation with motion, mapping, and camera

set -e

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}🚀 Starting Dojo Robot Full Simulation${NC}"
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
SLAM=${4:-"true"}

echo -e "${GREEN}🎮 Launch Configuration:${NC}"
echo "  World: $WORLD"
echo "  Gazebo GUI: $GUI"
echo "  RViz: $RVIZ"
echo "  SLAM Mapping: $SLAM"
echo ""

echo -e "${BLUE}🌍 Launching full simulation...${NC}"
echo ""
echo -e "${YELLOW}Controls:${NC}"
echo "  - Use the teleop terminal window to drive the robot"
echo "  - Watch the map build in RViz as you drive around"
echo "  - Camera feed is shown in RViz Image panel"
echo "  - LiDAR data appears as red points"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop simulation${NC}"
echo ""

# Launch the full simulation
ros2 launch robot_gazebo full_simulation.launch.py \
    world:=$WORLD \
    gui:=$GUI \
    rviz:=$RVIZ \
    slam:=$SLAM \
    use_teleop:=true \
    camera_view:=false