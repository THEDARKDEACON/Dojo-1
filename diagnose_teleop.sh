#!/bin/bash

echo "=== Teleop Diagnostics ==="
echo "This script helps diagnose teleop control issues"
echo ""

echo "1. Checking if controllers are loaded..."
ros2 control list_controllers 2>/dev/null || echo "Controller manager not available"

echo ""
echo "2. Checking available topics..."
echo "Looking for cmd_vel topics:"
ros2 topic list | grep cmd_vel || echo "No cmd_vel topics found"

echo ""
echo "3. Checking diff_drive_controller status..."
ros2 control list_controllers 2>/dev/null | grep diff_drive || echo "diff_drive_controller not found"

echo ""
echo "4. Checking joint states..."
echo "Available joint state topics:"
ros2 topic list | grep joint || echo "No joint topics found"

echo ""
echo "5. Testing topic publishing..."
echo "Publishing test command to /diff_drive_controller/cmd_vel..."
timeout 2s ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null && echo "✓ Successfully published test command" || echo "✗ Failed to publish test command"

echo ""
echo "6. Checking if robot is receiving commands..."
echo "Listening to /diff_drive_controller/cmd_vel for 3 seconds..."
timeout 3s ros2 topic echo /diff_drive_controller/cmd_vel --once 2>/dev/null && echo "✓ Topic is active" || echo "✗ No data on topic"

echo ""
echo "=== Troubleshooting Tips ==="
echo "If teleop is not working:"
echo "1. Make sure the simulation is fully loaded (wait ~10 seconds after launch)"
echo "2. Check that the teleop terminal window has focus"
echo "3. Try pressing keys in the teleop window: w/a/s/d/x"
echo "4. Verify controllers are loaded: ros2 control list_controllers"
echo "5. Test manual command: ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}' --once"