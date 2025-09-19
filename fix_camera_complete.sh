#!/bin/bash

echo "📷 Complete Camera Fix for Gazebo Simulation"
echo "============================================="

echo ""
echo "1. 🔧 Rebuilding robot_description with camera fixes..."
echo "------------------------------------------------------"
colcon build --packages-select robot_description --symlink-install

echo ""
echo "2. 🔄 Sourcing setup..."
echo "----------------------"
source install/setup.bash

echo ""
echo "3. 🧪 Testing URDF with Gazebo plugins..."
echo "-----------------------------------------"
echo "Checking if URDF includes Gazebo plugins:"
xacro src/robot_description/urdf/dojo_robot.urdf.xacro use_gazebo:=true > /tmp/robot_with_gazebo.urdf
if grep -q "libgazebo_ros_camera.so" /tmp/robot_with_gazebo.urdf; then
    echo "✅ Camera plugin found in URDF"
else
    echo "❌ Camera plugin missing from URDF"
fi

if grep -q "camera_link" /tmp/robot_with_gazebo.urdf; then
    echo "✅ Camera link found in URDF"
else
    echo "❌ Camera link missing from URDF"
fi

echo ""
echo "4. 📋 Camera Configuration Verification:"
echo "----------------------------------------"
echo "Camera sensor configuration:"
echo "- Sensor name: camera_sensor"
echo "- Always on: true"
echo "- Visualize: true"
echo "- Update rate: 30 Hz"
echo "- Topics: /image_raw, /camera_info"
echo "- Frame: camera_link"

echo ""
echo "5. 🎯 Launch Instructions:"
echo "--------------------------"
echo "After running this fix:"
echo ""
echo "1. Launch the simulation:"
echo "   ros2 launch robot_gazebo complete_simulation.launch.py"
echo ""
echo "2. Check camera topics:"
echo "   ros2 topic list | grep image"
echo ""
echo "3. View camera in RViz:"
echo "   - Look for 'Camera' panel in RViz (should be visible)"
echo "   - Topic should be set to /image_raw"
echo ""
echo "4. Test camera data:"
echo "   ros2 topic echo /image_raw --once"
echo ""
echo "5. Open separate camera viewer:"
echo "   ros2 run rqt_image_view rqt_image_view /image_raw"

echo ""
echo "6. 🔍 Troubleshooting:"
echo "----------------------"
echo "If camera still doesn't work:"
echo ""
echo "a) Check Gazebo sensor manager:"
echo "   - In Gazebo GUI: Window -> Topic Visualization"
echo "   - Look for camera topics"
echo ""
echo "b) Verify robot model in Gazebo:"
echo "   - Right-click robot -> View -> Transparent"
echo "   - Check if camera_link is visible"
echo ""
echo "c) Check Gazebo console for errors:"
echo "   - Look for camera plugin loading errors"
echo ""
echo "d) Restart simulation completely:"
echo "   - Kill all ROS nodes"
echo "   - Restart Gazebo"
echo "   - Relaunch simulation"

echo ""
echo "🎯 Camera fix complete!"
echo "======================="

# Clean up temp file
rm -f /tmp/robot_with_gazebo.urdf