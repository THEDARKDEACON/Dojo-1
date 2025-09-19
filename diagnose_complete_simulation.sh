#!/bin/bash

echo "🔍 Diagnosing Complete Simulation Issues..."
echo "=========================================="

echo ""
echo "1. 🤖 Checking ROS2 Topics:"
echo "----------------------------"
echo "Available topics:"
ros2 topic list | head -20

echo ""
echo "2. 📷 Checking Camera Topics:"
echo "-----------------------------"
ros2 topic list | grep -E "(image|camera)" || echo "❌ No camera topics found"

echo ""
echo "3. 🗺️ Checking Map Topics:"
echo "--------------------------"
ros2 topic list | grep -E "(map|slam)" || echo "❌ No map topics found"

echo ""
echo "4. 🎮 Checking Control Topics:"
echo "------------------------------"
ros2 topic list | grep -E "(cmd_vel|diff_drive)" || echo "❌ No control topics found"

echo ""
echo "5. 📡 Checking Topic Info:"
echo "--------------------------"
echo "Camera topic info:"
ros2 topic info /image_raw 2>/dev/null || echo "❌ /image_raw topic not available"

echo ""
echo "Control topic info:"
ros2 topic info /diff_drive_controller/cmd_vel_unstamped 2>/dev/null || echo "❌ /diff_drive_controller/cmd_vel_unstamped topic not available"

echo ""
echo "6. 🔧 Checking robot_perception:"
echo "--------------------------------"
ls -la install/lib/robot_perception/ 2>/dev/null || echo "❌ robot_perception executables not found"

echo ""
echo "7. 📊 Checking Active Nodes:"
echo "----------------------------"
ros2 node list | head -10

echo ""
echo "🎯 Diagnosis Complete!"
echo "======================"