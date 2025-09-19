#!/bin/bash

echo "🗺️ World Testing Script for Dojo Robot"
echo "======================================="

echo ""
echo "Available worlds for testing:"
echo "----------------------------"

# Define interesting worlds for robot testing
declare -A WORLDS
WORLDS[1]="empty.world - Basic empty environment"
WORLDS[2]="dojo_world.world - Default dojo environment"
WORLDS[3]="demo_world.world - Demo environment with obstacles"
WORLDS[4]="warehouse.world - Warehouse environment"
WORLDS[5]="office_small.world - Small office environment"
WORLDS[6]="house.world - House environment"
WORLDS[7]="neighborhood.world - Neighborhood environment"
WORLDS[8]="simple_env_1.world - Simple test environment 1"
WORLDS[9]="simple_env_2.world - Simple test environment 2"
WORLDS[10]="simple_env_3.world - Simple test environment 3"
WORLDS[11]="test_zone.world - Test zone environment"
WORLDS[12]="outdoor.world - Outdoor environment"

# Display options
for key in $(echo ${!WORLDS[@]} | tr ' ' '\n' | sort -n); do
    echo "$key) ${WORLDS[$key]}"
done

echo ""
echo "Usage examples:"
echo "--------------"
echo "ros2 launch robot_gazebo complete_simulation.launch.py world:=warehouse.world"
echo "ros2 launch robot_gazebo complete_simulation.launch.py world:=office_small.world"
echo "ros2 launch robot_gazebo complete_simulation.launch.py world:=house.world"

echo ""
echo "🎯 Quick Launch Commands:"
echo "========================="

# Create quick launch functions
cat << 'EOF'

# Copy and paste these commands to test different worlds:

# 1. Empty world (good for basic testing)
ros2 launch robot_gazebo complete_simulation.launch.py world:=empty.world

# 2. Warehouse (good for SLAM testing)
ros2 launch robot_gazebo complete_simulation.launch.py world:=warehouse.world

# 3. Office (indoor navigation)
ros2 launch robot_gazebo complete_simulation.launch.py world:=office_small.world

# 4. House (residential environment)
ros2 launch robot_gazebo complete_simulation.launch.py world:=house.world

# 5. Neighborhood (outdoor navigation)
ros2 launch robot_gazebo complete_simulation.launch.py world:=neighborhood.world

# 6. Simple test environments
ros2 launch robot_gazebo complete_simulation.launch.py world:=simple_env_1.world
ros2 launch robot_gazebo complete_simulation.launch.py world:=simple_env_2.world
ros2 launch robot_gazebo complete_simulation.launch.py world:=simple_env_3.world

EOF

echo ""
echo "🔧 Pro Tips:"
echo "============"
echo "• Use 'empty.world' for basic robot testing"
echo "• Use 'warehouse.world' or 'office_small.world' for SLAM testing"
echo "• Use 'house.world' or 'neighborhood.world' for navigation testing"
echo "• Check RViz map panel to see SLAM building the map"
echo "• Use teleop (i/j/k/l keys) to drive around and build maps"