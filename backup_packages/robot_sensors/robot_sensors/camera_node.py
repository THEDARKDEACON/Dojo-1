#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraNode(Node):
    """
    ROS 2 node for camera interface.
    Uses camera_ros package to interface with various camera types.
    """
    
    def __init__(self):
        super().__init__('camera_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('camera_name', 'camera')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        
        # Get parameters
        self.camera_name = self.get_parameter('camera_name').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # Create publisher for raw images
        self.image_pub = self.create_publisher(
            Image, 
            f'/{self.camera_name}/image_raw', 
            10
        )
        
        # Create timer for publishing frames
        timer_period = 1.0 / self.get_parameter('fps').value
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Camera node started: {self.camera_name}')
    
    def timer_callback(self):
        """Publish camera frames."""
        try:
            # TODO: Replace with actual camera capture using camera_ros
            # For now, we'll publish a test pattern
            width = self.get_parameter('width').value
            height = self.get_parameter('height').value
            
            # Create a test pattern
            frame = np.zeros((height, width, 3), dtype=np.uint8)
            cv2.putText(frame, f'{width}x{height}', 
                       (width//4, height//2), 
                       cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
            
            # Convert to ROS message and publish
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.image_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in camera capture: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
