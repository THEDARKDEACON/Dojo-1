#!/bin/bash

echo "📷 Fixing Camera Issues..."
echo "=========================="

echo ""
echo "1. 🔧 Rebuilding robot_description with camera fixes..."
echo "------------------------------------------------------"
colcon build --packages-select robot_description --symlink-install

echo ""
echo "2. 🔄 Sourcing setup..."
echo "----------------------"
source install/setup.bash

echo ""
echo "3. 🧪 Testing camera topics (run this after launching simulation)..."
echo "--------------------------------------------------------------------"
echo "To test camera after launching simulation, run:"
echo ""
echo "# Check if camera topics exist:"
echo "ros2 topic list | grep -E '(image|camera)'"
echo ""
echo "# Check camera topic info:"
echo "ros2 topic info /image_raw"
echo ""
echo "# View camera feed in terminal:"
echo "ros2 topic echo /image_raw --once"
echo ""
echo "# Open camera viewer:"
echo "ros2 run rqt_image_view rqt_image_view /image_raw"

echo ""
echo "4. 📋 Camera Configuration Summary:"
echo "-----------------------------------"
echo "✅ Camera sensor: always_on=true, visualize=true"
echo "✅ Camera topics: /image_raw, /camera_info"
echo "✅ Camera frame: camera_link"
echo "✅ Update rate: 30 Hz"
echo "✅ Resolution: 640x480"

echo ""
echo "🎯 Camera fixes applied!"
echo "========================"
echo ""
echo "Next steps:"
echo "1. Launch simulation: ros2 launch robot_gazebo complete_simulation.launch.py"
echo "2. Check camera in RViz Image panel"
echo "3. If still not working, check Gazebo sensor manager"