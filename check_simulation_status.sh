#!/bin/bash
echo "🔍 Simulation Status Check"
echo "=========================="
echo ""
echo "1. 📡 Active ROS2 Topics:"
echo "-------------------------"
ros2 topic list | head -20
echo ""
echo "2. 📷 Camera Topics:"
echo "-------------------"
CAMERA_TOPICS=$(ros2 topic list | grep -E "(image|camera)")
if [ -n "$CAMERA_TOPICS" ]; then
    echo "$CAMERA_TOPICS"
    echo ""
    echo "Camera topic info:"
    ros2 topic info /image_raw 2>/dev/null || echo "❌ /image_raw not available"
else
    echo "❌ No camera topics found"
fi
echo ""
echo "3. 🗺️ SLAM Topics:"
echo "------------------"
SLAM_TOPICS=$(ros2 topic list | grep -E "(map|slam)")
if [ -n "$SLAM_TOPICS" ]; then
    echo "$SLAM_TOPICS"
else
    echo "❌ No SLAM topics found"
fi
echo ""
echo "4. 🎮 Control Topics:"
echo "--------------------"
CONTROL_TOPICS=$(ros2 topic list | grep -E "(cmd_vel|diff_drive)")
if [ -n "$CONTROL_TOPICS" ]; then
    echo "$CONTROL_TOPICS"
else
    echo "❌ No control topics found"
fi
echo ""
echo "5. 🤖 Active Nodes:"
echo "------------------"
ros2 node list | head -10
echo ""
echo "6. 📊 Topic Publishing Status:"
echo "------------------------------"
echo "Checking if topics are publishing data..."
if ros2 topic list | grep -q "/image_raw"; then
    echo -n "Camera (/image_raw): "
    timeout 3s ros2 topic hz /image_raw 2>/dev/null | head -1 || echo "Not publishing"
fi
if ros2 topic list | grep -q "/scan"; then
    echo -n "LiDAR (/scan): "
    timeout 3s ros2 topic hz /scan 2>/dev/null | head -1 || echo "Not publishing"
fi
if ros2 topic list | grep -q "/diff_drive_controller/cmd_vel_unstamped"; then
    echo "Control topic: ✅ Available"
else
    echo "Control topic: ❌ Missing"
fi
echo ""
echo "🎯 Status Check Complete!"