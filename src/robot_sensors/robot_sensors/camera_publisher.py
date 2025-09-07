#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import yaml

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        # Parameters
        self.declare_parameter('camera_id', 0)  # Default to first camera
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('camera_info_url', '')
        
        self.camera_id = self.get_parameter('camera_id').value
        self.frame_id = self.get_parameter('frame_id').value
        self.fps = self.get_parameter('fps').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        
        # Initialize OpenCV camera
        self.cap = cv2.VideoCapture(self.camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        if not self.cap.isOpened():
            self.get_logger().error(f'Could not open camera with ID {self.camera_id}')
            return
        
        # Setup CV bridge
        self.bridge = CvBridge()
        
        # Setup publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        
        # Load camera info if available
        self.camera_info = self.load_camera_info()
        
        # Create timer for publishing frames
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Camera publisher started with {self.width}x{self.height} @ {self.fps} FPS')
    
    def load_camera_info(self):
        """Load camera info from URL if provided, otherwise return default."""
        camera_info_url = self.get_parameter('camera_info_url').value
        if not camera_info_url:
            # Create a default camera info
            camera_info = CameraInfo()
            camera_info.header.frame_id = self.frame_id
            camera_info.width = self.width
            camera_info.height = self.height
            # Identity matrix (no distortion, focal length = 1.0, principal point at center)
            camera_info.k = [1.0, 0.0, float(self.width) / 2.0,
                           0.0, 1.0, float(self.height) / 2.0,
                           0.0, 0.0, 1.0]
            camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
            camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # Identity
            camera_info.p = [1.0, 0.0, float(self.width) / 2.0, 0.0,
                          0.0, 1.0, float(self.height) / 2.0, 0.0,
                          0.0, 0.0, 1.0, 0.0]
            return camera_info
        
        # TODO: Implement loading from URL (file:// or package://)
        self.get_logger().warn('Loading camera info from URL not yet implemented')
        return None
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return
        
        # Convert to ROS2 message and publish
        try:
            # Convert BGR to RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Publish image
            img_msg = self.bridge.cv2_to_imgmsg(rgb_frame, 'rgb8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = self.frame_id
            self.image_pub.publish(img_msg)
            
            # Publish camera info if available
            if self.camera_info is not None:
                self.camera_info.header.stamp = img_msg.header.stamp
                self.camera_info_pub.publish(self.camera_info)
                
        except Exception as e:
            self.get_logger().error(f'Error processing frame: {str(e)}')
    
    def __del__(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
