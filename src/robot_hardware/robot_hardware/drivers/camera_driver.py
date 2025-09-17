#!/usr/bin/env python3
"""
Unified Camera Driver for Dojo Robot
Consolidates camera functionality from robot_sensors and camera_ws
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import threading
import time

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import Header, String
from cv_bridge import CvBridge

class CameraDriver(Node):
    """Unified camera driver supporting multiple camera types"""
    
    def __init__(self):
        super().__init__('camera_driver')
        
        # Declare parameters
        self._declare_parameters()
        
        # Initialize camera
        self.camera = None
        self.camera_active = False
        self.cv_bridge = CvBridge()
        
        # Setup ROS interfaces
        self._setup_publishers()
        
        # Start camera thread
        self._start_camera()
        
        self.get_logger().info('Camera Driver initialized')
    
    def _declare_parameters(self):
        """Declare camera parameters"""
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('camera_name', 'camera')
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('compression_quality', 80)
        
    def _setup_publishers(self):
        """Setup ROS publishers"""
        self.image_pub = self.create_publisher(Image, 'image_raw', 10)
        self.compressed_pub = self.create_publisher(CompressedImage, 'image_raw/compressed', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera_info', 10)
        self.status_pub = self.create_publisher(String, 'camera_status', 10)
        
    def _start_camera(self):
        """Initialize and start camera capture"""
        try:
            camera_id = self.get_parameter('camera_id').value
            self.camera = cv2.VideoCapture(camera_id)
            
            if not self.camera.isOpened():
                raise Exception(f"Cannot open camera {camera_id}")
            
            # Set camera properties
            width = self.get_parameter('width').value
            height = self.get_parameter('height').value
            fps = self.get_parameter('fps').value
            
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self.camera.set(cv2.CAP_PROP_FPS, fps)
            
            self.camera_active = True
            
            # Start capture thread
            self.capture_thread = threading.Thread(target=self._capture_loop)
            self.capture_thread.daemon = True
            self.capture_thread.start()
            
            self.get_logger().info(f'Camera {camera_id} started: {width}x{height} @ {fps}fps')
            self._publish_status('ACTIVE')
            
        except Exception as e:
            self.get_logger().error(f'Failed to start camera: {e}')
            self._publish_status('ERROR')
    
    def _capture_loop(self):
        """Main camera capture loop"""
        fps = self.get_parameter('fps').value
        frame_time = 1.0 / fps
        
        while rclpy.ok() and self.camera_active:
            start_time = time.time()
            
            if self.camera and self.camera.isOpened():
                ret, frame = self.camera.read()
                
                if ret:
                    self._publish_frame(frame)
                else:
                    self.get_logger().warn('Failed to capture frame')
                    
            # Maintain frame rate
            elapsed = time.time() - start_time
            sleep_time = max(0, frame_time - elapsed)
            time.sleep(sleep_time)    

    def _publish_frame(self, frame):
        """Publish camera frame as ROS messages"""
        timestamp = self.get_clock().now().to_msg()
        frame_id = self.get_parameter('frame_id').value
        
        # Create header
        header = Header()
        header.stamp = timestamp
        header.frame_id = frame_id
        
        try:
            # Publish raw image
            image_msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image_msg.header = header
            self.image_pub.publish(image_msg)
            
            # Publish compressed image if enabled
            if self.get_parameter('publish_compressed').value:
                self._publish_compressed(frame, header)
                
            # Publish camera info
            self._publish_camera_info(header)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish frame: {e}')
    
    def _publish_compressed(self, frame, header):
        """Publish compressed image"""
        try:
            quality = self.get_parameter('compression_quality').value
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality]
            
            success, encoded_image = cv2.imencode('.jpg', frame, encode_params)
            
            if success:
                compressed_msg = CompressedImage()
                compressed_msg.header = header
                compressed_msg.format = 'jpeg'
                compressed_msg.data = encoded_image.tobytes()
                
                self.compressed_pub.publish(compressed_msg)
                
        except Exception as e:
            self.get_logger().error(f'Failed to publish compressed image: {e}')
    
    def _publish_camera_info(self, header):
        """Publish camera calibration info"""
        camera_info = CameraInfo()
        camera_info.header = header
        
        # Basic camera info (should be calibrated for real use)
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        
        camera_info.width = width
        camera_info.height = height
        camera_info.distortion_model = 'plumb_bob'
        
        # Placeholder calibration values (replace with actual calibration)
        camera_info.k = [
            width/2, 0.0, width/2,
            0.0, height/2, height/2,
            0.0, 0.0, 1.0
        ]
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [
            width/2, 0.0, width/2, 0.0,
            0.0, height/2, height/2, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        self.camera_info_pub.publish(camera_info)
    
    def _publish_status(self, status):
        """Publish camera status"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)
    
    def cleanup(self):
        """Cleanup camera resources"""
        self.camera_active = False
        if self.camera:
            self.camera.release()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        camera_driver = CameraDriver()
        rclpy.spin(camera_driver)
    except KeyboardInterrupt:
        pass
    finally:
        if 'camera_driver' in locals():
            camera_driver.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()