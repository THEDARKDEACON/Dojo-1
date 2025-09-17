#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        
        # Declare parameters
        self.declare_parameter('use_sim_time', False)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('angle_compensate', True)
        self.declare_parameter('scan_mode', 'Sensitivity')
        
        # Get parameters
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        # Create publisher
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        
        # Create timer for publishing dummy data (replace with actual lidar driver)
        self.timer = self.create_timer(0.1, self.publish_scan)
        
        self.get_logger().info('LiDAR node started (dummy mode)')
    
    def publish_scan(self):
        """Publish a dummy laser scan for testing"""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        
        # Dummy scan parameters
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = math.pi / 180.0  # 1 degree
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.15
        scan.range_max = 12.0
        
        # Create dummy ranges (circle of obstacles at 2m)
        num_readings = int((scan.angle_max - scan.angle_min) / scan.angle_increment)
        scan.ranges = [2.0] * num_readings
        scan.intensities = [100.0] * num_readings
        
        self.scan_pub.publish(scan)

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()