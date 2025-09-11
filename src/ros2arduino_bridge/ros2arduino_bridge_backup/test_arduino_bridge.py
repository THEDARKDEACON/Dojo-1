#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math

class TestArduinoBridge(Node):
    def __init__(self):
        super().__init__('test_arduino_bridge')
        
        # Parameters
        self.declare_parameter('test_duration', 10.0)
        self.test_duration = self.get_parameter('test_duration').value
        
        # Publishers and Subscribers
        self.cmd_pub = self.create_publisher(JointState, 'wheel_commands', 10)
        self.encoder_sub = self.create_subscription(
            JointState,
            'wheel_encoders',
            self.encoder_callback,
            10
        )
        
        # Test state
        self.test_start_time = self.get_clock().now()
        self.test_complete = False
        self.encoder_updates = 0
        
        # Start test sequence
        self.timer = self.create_timer(0.1, self.test_sequence)
        
    def encoder_callback(self, msg):
        """Handle encoder updates."""
        self.encoder_updates += 1
        self.get_logger().info(
            f"Encoder update - Left: {msg.position[0]}, Right: {msg.position[1]}"
        )
    
    def test_sequence(self):
        """Run test sequence."""
        current_time = (self.get_clock().now() - self.test_start_time).nanoseconds / 1e9
        
        if current_time > self.test_duration:
            if not self.test_complete:
                self.complete_test()
            return
        
        # Send test commands
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = ['left_wheel_joint', 'right_wheel_joint']
        
        # Test pattern: forward, turn right, turn left, stop
        if current_time < self.test_duration * 0.25:
            # Forward
            cmd.velocity = [0.5, 0.5]  # m/s
            self.get_logger().info("Test: Moving forward")
        elif current_time < self.test_duration * 0.5:
            # Turn right
            cmd.velocity = [0.5, -0.5]
            self.get_logger().info("Test: Turning right")
        elif current_time < self.test_duration * 0.75:
            # Turn left
            cmd.velocity = [-0.5, 0.5]
            self.get_logger().info("Test: Turning left")
        else:
            # Stop
            cmd.velocity = [0.0, 0.0]
            self.get_logger().info("Test: Stopping")
        
        self.cmd_pub.publish(cmd)
    
    def complete_test(self):
        """Complete the test and print results."""
        self.test_complete = True
        self.get_logger().info("\n=== Test Complete ===")
        self.get_logger().info(f"Test duration: {self.test_duration} seconds")
        self.get_logger().info(f"Encoder updates received: {self.encoder_updates}")
        self.get_logger().info("Test completed successfully!")
        
        # Shutdown after a short delay
        self.create_timer(1.0, lambda: rclpy.shutdown())

def main(args=None):
    rclpy.init(args=args)
    test_node = TestArduinoBridge()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
