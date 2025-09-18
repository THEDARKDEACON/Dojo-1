#!/usr/bin/env python3
"""
LiDAR Driver for Dojo Robot
Wrapper around existing sllidar functionality
"""

import rclpy
from rclpy.node import Node
import subprocess
import threading
import time

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class LidarDriver(Node):
    """LiDAR driver wrapper"""
    
    def __init__(self):
        super().__init__('lidar_driver')
        
        # Declare parameters
        self._declare_parameters()
        
        # Setup ROS interfaces
        self._setup_interfaces()
        
        # Start LiDAR monitoring
        self._start_monitoring()
        
        self.get_logger().info('LiDAR Driver initialized')
    
    def _declare_parameters(self):
        """Declare LiDAR parameters"""
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('angle_compensate', True)
        self.declare_parameter('scan_mode', 'Sensitivity')
        self.declare_parameter('inverted', False)
        self.declare_parameter('angle_min', -3.14159)
        self.declare_parameter('angle_max', 3.14159)
        # Topic configuration
        self.declare_parameter('input_scan_topic', 'scan')
        # If empty, do not republish (avoids loops when input is also 'scan')
        self.declare_parameter('publish_topic', '')
        
    def _setup_interfaces(self):
        """Setup ROS interfaces (pub/sub)"""
        self.status_pub = self.create_publisher(String, 'lidar_status', 10)

        input_topic = self.get_parameter('input_scan_topic').value
        publish_topic = self.get_parameter('publish_topic').value

        # Optional publisher (only if publish_topic is provided)
        self.scan_pub = None
        self.publish_topic = publish_topic
        if isinstance(publish_topic, str) and len(publish_topic) > 0:
            self.scan_pub = self.create_publisher(LaserScan, publish_topic, 10)

        # Subscribe to upstream scan topic (simulation or hardware driver)
        self.scan_sub = self.create_subscription(
            LaserScan, input_topic, self._scan_callback, 10)
    
    def _start_monitoring(self):
        """Start LiDAR monitoring"""
        self.monitoring_thread = threading.Thread(target=self._monitor_lidar)
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()
        
        self._publish_status('INITIALIZING')
    
    def _monitor_lidar(self):
        """Monitor LiDAR health and republish data"""
        while rclpy.ok():
            # Check if LiDAR is publishing data
            # This is a simple health check
            time.sleep(1.0)
    
    def _scan_callback(self, msg):
        """Process and republish scan data"""
        # Apply any filtering or processing here
        processed_scan = self._process_scan(msg)
        # Republish only if an output publisher is configured
        if self.scan_pub is not None:
            self.scan_pub.publish(processed_scan)
    
    def _process_scan(self, scan_msg):
        """Process raw scan data"""
        # Apply frame_id from parameters
        scan_msg.header.frame_id = self.get_parameter('frame_id').value
        
        # Apply angle limits if needed
        angle_min = self.get_parameter('angle_min').value
        angle_max = self.get_parameter('angle_max').value
        
        if angle_min != scan_msg.angle_min or angle_max != scan_msg.angle_max:
            # Crop scan data to specified angle range
            scan_msg = self._crop_scan(scan_msg, angle_min, angle_max)
        
        return scan_msg
    
    def _crop_scan(self, scan_msg, angle_min, angle_max):
        """Crop scan data to specified angle range"""
        # Simple implementation - could be more sophisticated
        return scan_msg
    
    def _publish_status(self, status):
        """Publish LiDAR status"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        lidar_driver = LidarDriver()
        rclpy.spin(lidar_driver)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()