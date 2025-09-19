#!/bin/bash

echo "🔍 Diagnosing Camera Issues..."
echo "=============================="

echo ""
echo "1. 📡 Checking Camera Topics:"
echo "-----------------------------"
echo "All topics:"
ros2 topic list | grep -E "(image|camera)" || echo "❌ No camera topics found"

echo ""
echo "2. 📊 Camera Topic Details:"
echo "---------------------------"
if ros2 topic list | grep -q "/image_raw"; then
    echo "✅ /image_raw topic exists"
    echo "Topic info:"
    ros2 topic info /image_raw
    echo ""
    echo "Topic type:"
    ros2 topic type /image_raw
    echo ""
    echo "Publishing rate:"
    timeout 5s ros2 topic hz /image_raw 2>/dev/null || echo "❌ Not publishing or very slow"
else
    echo "❌ /image_raw topic missing"
fi

echo ""
echo "3. 🤖 Checking Gazebo Sensors:"
echo "------------------------------"
echo "Active nodes:"
ros2 node list | grep -E "(gazebo|camera)" || echo "❌ No Gazebo camera nodes found"

echo ""
echo "4. 🔗 Checking TF Frames:"
echo "-------------------------"
echo "Camera frame (camera_link):"
ros2 run tf2_tools view_frames.py --help > /dev/null 2>&1 && echo "TF tools available" || echo "TF tools not available"
timeout 3s ros2 topic echo /tf_static --once 2>/dev/null | grep camera_link && echo "✅ camera_link frame exists" || echo "❌ camera_link frame missing"

echo ""
echo "5. 📷 Testing Camera Data:"
echo "-------------------------"
if ros2 topic list | grep -q "/image_raw"; then
    echo "Attempting to read one camera message..."
    timeout 5s ros2 topic echo /image_raw --once > /dev/null 2>&1 && echo "✅ Camera publishing data" || echo "❌ Camera not publishing data"
else
    echo "❌ Cannot test - topic doesn't exist"
fi

echo ""
echo "6. 🎮 Gazebo Sensor Status:"
echo "---------------------------"
echo "Check Gazebo GUI -> View -> Topic Visualization to see if camera sensor is active"
echo "Look for camera_link in the robot model in Gazebo"

echo ""
echo "🎯 Camera Diagnosis Complete!"
echo "============================="
echo ""
echo "Common fixes:"
echo "1. Rebuild robot_description: colcon build --packages-select robot_description"
echo "2. Check Gazebo plugins are loaded"
echo "3. Verify camera_link exists in robot model"
echo "4. Check RViz Image display topic is set to /image_raw"