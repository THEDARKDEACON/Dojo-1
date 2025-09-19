#!/bin/bash

echo "=== Controller Topic Analysis ==="
echo ""

echo "1. Checking controller info..."
ros2 control list_controllers -v

echo ""
echo "2. Checking all cmd_vel related topics..."
ros2 topic list | grep -E "(cmd_vel|twist)" || echo "No cmd_vel/twist topics found"

echo ""
echo "3. Checking topic info for diff_drive_controller topics..."
for topic in $(ros2 topic list | grep diff_drive_controller); do
    echo "Topic: $topic"
    ros2 topic info $topic
    echo ""
done

echo ""
echo "4. Testing different topic names..."
echo "Testing /diff_drive_controller/cmd_vel_unstamped..."
timeout 2s ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.1}}" 2>/dev/null && echo "✓ cmd_vel_unstamped works" || echo "✗ cmd_vel_unstamped failed"

echo "Testing /diff_drive_controller/cmd_vel..."
timeout 2s ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}" 2>/dev/null && echo "✓ cmd_vel works" || echo "✗ cmd_vel failed"

echo ""
echo "5. Checking what topics have subscribers..."
echo "Topics with subscribers:"
for topic in $(ros2 topic list | grep -E "(cmd_vel|twist)"); do
    info=$(ros2 topic info $topic 2>/dev/null)
    if echo "$info" | grep -q "Subscription count: [1-9]"; then
        echo "  $topic - HAS SUBSCRIBERS"
    else
        echo "  $topic - no subscribers"
    fi
done