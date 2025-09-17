#!/bin/bash

# Comprehensive Simulation Diagnostic Script
# Checks all aspects of the robot simulation

echo "🔍 Dojo Robot Simulation Diagnostics"
echo "===================================="

# Function to check if a topic exists and is publishing
check_topic() {
    local topic=$1
    local timeout=3
    
    echo -n "📡 Checking topic $topic... "
    
    if timeout $timeout ros2 topic hz $topic --once >/dev/null 2>&1; then
        echo "✅ ACTIVE"
        return 0
    else
        echo "❌ NO DATA"
        return 1
    fi
}

# Function to check if a node is running
check_node() {
    local node=$1
    echo -n "🤖 Checking node $node... "
    
    if ros2 node list | grep -q "$node"; then
        echo "✅ RUNNING"
        return 0
    else
        echo "❌ NOT FOUND"
        return 1
    fi
}

# Function to check transforms
check_transforms() {
    echo "🔗 Checking TF transforms..."
    
    # Check if basic transforms exist
    local frames=("odom" "base_link" "base_footprint" "laser")
    
    for frame in "${frames[@]}"; do
        echo -n "   Frame $frame... "
        if ros2 run tf2_ros tf2_echo odom $frame --timeout 2 >/dev/null 2>&1; then
            echo "✅ OK"
        else
            echo "❌ MISSING"
        fi
    done
}

echo ""
echo "1️⃣ CHECKING ROS2 NODES"
echo "----------------------"
check_node "gazebo"
check_node "robot_state_publisher"
check_node "controller_manager"
check_node "slam_toolbox"
check_node "control_manager"

echo ""
echo "2️⃣ CHECKING TOPICS"
echo "------------------"
check_topic "/odom"
check_topic "/cmd_vel"
check_topic "/scan"
check_topic "/image_raw"
check_topic "/joint_states"

echo ""
echo "3️⃣ CHECKING CONTROLLERS"
echo "-----------------------"
echo "📋 Available controllers:"
ros2 control list_controllers 2>/dev/null || echo "❌ Controller manager not responding"

echo ""
echo "4️⃣ CHECKING TRANSFORMS"
echo "----------------------"
check_transforms

echo ""
echo "5️⃣ CHECKING GAZEBO MODELS"
echo "-------------------------"
echo -n "🏗️ Robot model in Gazebo... "
if ros2 service call /gazebo/get_model_list gazebo_msgs/srv/GetModelList >/dev/null 2>&1; then
    if ros2 service call /gazebo/get_model_list gazebo_msgs/srv/GetModelList | grep -q "dojo_robot"; then
        echo "✅ LOADED"
    else
        echo "❌ NOT FOUND"
    fi
else
    echo "❌ GAZEBO NOT RESPONDING"
fi

echo ""
echo "6️⃣ DIAGNOSTIC SUMMARY"
echo "---------------------"

# Count issues
issues=0

# Check critical topics
critical_topics=("/odom" "/cmd_vel" "/scan")
for topic in "${critical_topics[@]}"; do
    if ! timeout 2 ros2 topic hz $topic --once >/dev/null 2>&1; then
        ((issues++))
    fi
done

if [ $issues -eq 0 ]; then
    echo "🎉 All systems appear to be working!"
    echo ""
    echo "🎮 Try controlling the robot:"
    echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_manual"
else
    echo "⚠️  Found $issues critical issues"
    echo ""
    echo "🔧 TROUBLESHOOTING STEPS:"
    echo "1. Restart the simulation: Ctrl+C and run ./run_full_simulation.sh"
    echo "2. Check Gazebo is running: ps aux | grep gazebo"
    echo "3. Manually spawn controllers:"
    echo "   ros2 run controller_manager spawner joint_state_broadcaster"
    echo "   ros2 run controller_manager spawner diff_drive_controller"
    echo "4. Check for error messages in the terminal"
fi

echo ""
echo "📊 USEFUL MONITORING COMMANDS:"
echo "   ros2 topic list                    # See all topics"
echo "   ros2 node list                     # See all nodes"
echo "   ros2 control list_controllers      # See controller status"
echo "   ros2 run tf2_tools view_frames     # Generate TF tree"
echo "   ros2 topic echo /diagnostics       # System health"