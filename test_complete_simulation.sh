#!/bin/bash

echo "🧪 Testing Complete Simulation..."
echo "================================="

echo ""
echo "1. 📡 Checking Essential Topics:"
echo "--------------------------------"

# Check camera topic
echo -n "Camera topic (/image_raw): "
if ros2 topic list | grep -q "/image_raw"; then
    echo "✅ Available"
    echo "  - Publishing rate: $(ros2 topic hz /image_raw --window 5 2>/dev/null | head -1 || echo 'Not publishing')"
else
    echo "❌ Missing"
fi

# Check map topic
echo -n "Map topic (/map): "
if ros2 topic list | grep -q "/map"; then
    echo "✅ Available"
else
    echo "❌ Missing"
fi

# Check control topic
echo -n "Control topic (/diff_drive_controller/cmd_vel_unstamped): "
if ros2 topic list | grep -q "/diff_drive_controller/cmd_vel_unstamped"; then
    echo "✅ Available"
else
    echo "❌ Missing"
fi

# Check scan topic
echo -n "LiDAR topic (/scan): "
if ros2 topic list | grep -q "/scan"; then
    echo "✅ Available"
    echo "  - Publishing rate: $(ros2 topic hz /scan --window 5 2>/dev/null | head -1 || echo 'Not publishing')"
else
    echo "❌ Missing"
fi

echo ""
echo "2. 🤖 Checking Active Nodes:"
echo "----------------------------"
echo "SLAM node:"
ros2 node list | grep slam || echo "❌ SLAM node not running"

echo "Perception node:"
ros2 node list | grep perception || echo "❌ Perception node not running"

echo "Control manager:"
ros2 node list | grep control || echo "❌ Control manager not running"

echo ""
echo "3. 🎮 Testing Teleop Connection:"
echo "--------------------------------"
echo "Publishing test command to teleop topic..."
timeout 2s ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null && echo "✅ Teleop topic accepts commands" || echo "❌ Teleop topic not responding"

echo ""
echo "4. 📊 Topic Summary:"
echo "--------------------"
echo "Total topics: $(ros2 topic list | wc -l)"
echo "Camera-related: $(ros2 topic list | grep -E '(image|camera)' | wc -l)"
echo "Control-related: $(ros2 topic list | grep -E '(cmd_vel|diff_drive)' | wc -l)"
echo "SLAM-related: $(ros2 topic list | grep -E '(map|slam)' | wc -l)"

echo ""
echo "🎯 Test Complete!"
echo "=================="