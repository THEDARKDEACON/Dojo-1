#!/bin/bash

# Test script to verify SLAM fix
# This script will help diagnose if the frame fix resolves the odometry issue

echo "🔧 Testing SLAM Configuration Fix"
echo "================================="

# Check if we're in the right directory
if [ ! -f "src/robot_gazebo/config/slam_config.yaml" ]; then
    echo "❌ Error: Not in ROS workspace root"
    exit 1
fi

echo "✅ Found SLAM config file"

# Verify the fix is applied
if grep -q "base_frame: base_link" src/robot_gazebo/config/slam_config.yaml; then
    echo "✅ SLAM config fixed: base_frame is now base_link"
else
    echo "❌ SLAM config not fixed: base_frame is still base_footprint"
    exit 1
fi

# Check ROS2 control config for consistency
if grep -q "base_frame_id: base_link" src/robot_gazebo/config/ros2_control.yaml; then
    echo "✅ ROS2 control config consistent: base_frame_id is base_link"
else
    echo "⚠️  Warning: ROS2 control config might be inconsistent"
fi

# Check EKF config for consistency
if grep -q "base_link_frame: base_link" src/robot_gazebo/config/ekf_config.yaml; then
    echo "✅ EKF config consistent: base_link_frame is base_link"
else
    echo "⚠️  Warning: EKF config might be inconsistent"
fi

echo ""
echo "🚀 Ready to test! Run the simulation:"
echo "   ./run_full_simulation.sh"
echo ""
echo "📊 Monitor SLAM status with:"
echo "   ros2 topic echo /slam_toolbox/feedback"
echo "   ros2 topic hz /odom"
echo "   ros2 run tf2_tools view_frames"
echo ""
echo "🎮 Control the robot with:"
echo "   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_manual"