#!/bin/bash

echo "🔧 Fixing Complete Simulation Issues..."
echo "======================================="

echo ""
echo "1. 🔗 Fixing robot_perception symlinks..."
echo "-----------------------------------------"
# Create the robot_perception directory in install/lib if it doesn't exist
mkdir -p install/lib/robot_perception

# Create symlinks for the executables
if [ -f "src/robot_perception/robot_perception/object_detector.py" ]; then
    echo "Creating symlink for object_detector..."
    ln -sf "$(pwd)/src/robot_perception/robot_perception/object_detector.py" install/lib/robot_perception/object_detector
    chmod +x install/lib/robot_perception/object_detector
    echo "✅ object_detector symlink created"
else
    echo "❌ object_detector.py not found"
fi

echo ""
echo "2. 🏗️ Rebuilding packages..."
echo "-----------------------------"
colcon build --packages-select robot_gazebo robot_perception robot_control --symlink-install

echo ""
echo "3. 🔄 Sourcing setup..."
echo "----------------------"
source install/setup.bash

echo ""
echo "4. ✅ Verification:"
echo "------------------"
echo "robot_perception executables:"
ls -la install/lib/robot_perception/ 2>/dev/null || echo "Directory not found"

echo ""
echo "🎯 Complete simulation fixes applied!"
echo "====================================="
echo ""
echo "Now you can run:"
echo "  ros2 launch robot_gazebo complete_simulation.launch.py"
echo ""
echo "Expected features:"
echo "  ✅ LiDAR visualization in RViz"
echo "  ✅ Camera feed in RViz Image panel"
echo "  ✅ SLAM map building in RViz Map panel"
echo "  ✅ Teleop control with correct topic remapping"
echo "  ✅ Robot movement with i/j/k/l keys in teleop terminal"