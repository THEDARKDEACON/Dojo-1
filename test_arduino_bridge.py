#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time
import sys
import serial
import json

class ArduinoBridgeTester(Node):
    def __init__(self):
        super().__init__('arduino_bridge_tester')
        self.get_logger().info("Arduino Bridge Tester Started")
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.encoder_sub = self.create_subscription(
            JointState,
            '/wheel/encoders',
            self.encoder_callback,
            10
        )
        
        # Test parameters
        self.test_duration = 10.0  # seconds
        self.test_start_time = self.get_clock().now()
        self.encoder_updates = 0
        self.last_encoder_left = 0
        self.last_encoder_right = 0
        
        # Start test sequence
        self.timer = self.create_timer(0.1, self.test_sequence)
    
    def encoder_callback(self, msg):
        """Handle encoder updates."""
        self.encoder_updates += 1
        self.last_encoder_left = msg.position[0] if len(msg.position) > 0 else 0
        self.last_encoder_right = msg.position[1] if len(msg.position) > 1 else 0
    
    def test_sequence(self):
        """Run test sequence."""
        current_time = (self.get_clock().now() - self.test_start_time).nanoseconds / 1e9
        
        if current_time > self.test_duration:
            self.complete_test()
            return
        
        # Send test commands based on time
        cmd = Twist()
        
        if current_time < self.test_duration * 0.25:
            # Forward
            cmd.linear.x = 0.2
            self.get_logger().info("Test: Moving forward")
        elif current_time < self.test_duration * 0.5:
            # Turn right
            cmd.angular.z = -0.5
            self.get_logger().info("Test: Turning right")
        elif current_time < self.test_duration * 0.75:
            # Turn left
            cmd.angular.z = 0.5
            self.get_logger().info("Test: Turning left")
        else:
            # Stop
            self.get_logger().info("Test: Stopping")
        
        self.cmd_pub.publish(cmd)
    
    def complete_test(self):
        """Complete the test and print results."""
        self.get_logger().info("\n=== Test Complete ===")
        self.get_logger().info(f"Test duration: {self.test_duration} seconds")
        self.get_logger().info(f"Encoder updates received: {self.encoder_updates}")
        self.get_logger().info(f"Final encoder values - Left: {self.last_encoder_left}, Right: {self.last_encoder_right}")
        
        # Shutdown
        self.get_logger().info("Shutting down...")
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    tester = ArduinoBridgeTester()
    rclpy.spin(tester)

if __name__ == '__main__':
    main()
