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
        """Initialize and start camera capture with enhanced error handling"""
        camera_id = self.get_parameter('camera_id').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        
        self.get_logger().info(f'Attempting to initialize camera {camera_id} at {width}x{height} @ {fps}fps')
        
        try:
            # Try to open camera with retries
            max_retries = 3
            for attempt in range(max_retries):
                self.camera = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
                
                if not self.camera.isOpened():
                    if attempt == max_retries - 1:
                        raise RuntimeError(f"Failed to open camera {camera_id} after {max_retries} attempts")
                    self.get_logger().warning(f"Attempt {attempt + 1} failed to open camera, retrying...")
                    time.sleep(1)
                    continue
                    
                # Set camera properties
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                self.camera.set(cv2.CAP_PROP_FPS, fps)
                
                # Verify settings
                actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
                actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
                
                if actual_width != width or actual_height != height:
                    self.get_logger().warning(
                        f'Camera resolution set to {actual_width}x{actual_height} '
                        f'instead of requested {width}x{height}'
                    )
                
                self.camera_active = True
                
                # Start capture thread
                self.capture_thread = threading.Thread(target=self._capture_loop)
                self.capture_thread.daemon = True
                self.capture_thread.start()
                
                self.get_logger().info(
                    f'Camera {camera_id} started: {actual_width}x{actual_height} @ {actual_fps:.1f}fps'
                )
                self._publish_status('ACTIVE')
                return
                
        except Exception as e:
            self.get_logger().error(f'Failed to start camera: {str(e)}')
            self._publish_status('ERROR')
            if self.camera is not None:
                self.camera.release()
                self.camera = None
    
    def _capture_loop(self):
        """Main camera capture loop with enhanced error handling"""
        fps = self.get_parameter('fps').value
        frame_time = 1.0 / fps
        consecutive_errors = 0
        max_consecutive_errors = 5
        
        while rclpy.ok() and self.camera_active:
            start_time = time.time()
            
            try:
                if self.camera is None or not self.camera.isOpened():
                    self.get_logger().error('Camera is not open, attempting to reopen...')
                    self._start_camera()
                    time.sleep(1)  # Wait before next attempt
                    continue
                
                # Read frame
                ret, frame = self.camera.read()
                
                if not ret:
                    consecutive_errors += 1
                    error_msg = f'Failed to capture frame (attempt {consecutive_errors}/{max_consecutive_errors})'
                    self.get_logger().warning(error_msg)
                    
                    if consecutive_errors >= max_consecutive_errors:
                        self.get_logger().error('Max consecutive capture errors reached, resetting camera...')
                        if self.camera is not None:
                            self.camera.release()
                            self.camera = None
                        time.sleep(1)
                        consecutive_errors = 0
                    continue
                
                # Reset error counter on successful capture
                consecutive_errors = 0
                
                # Publish the frame
                self._publish_frame(frame)
                
            except Exception as e:
                self.get_logger().error(f'Error in capture loop: {str(e)}')
                time.sleep(1)  # Prevent tight loop on error
                
            # Maintain frame rate
            elapsed = time.time() - start_time
            sleep_time = max(0, frame_time - elapsed)
            if sleep_time > 0:
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