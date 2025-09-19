#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Point

class CameraProcessor(Node):
    """
    Process camera images for basic computer vision tasks.
    
    Subscribes:
        /camera/image_raw - Raw camera image
        /camera/camera_info - Camera calibration info
        
    Publishes:
        /perception/image_processed - Processed image with visualizations
        /perception/object_position - Detected object positions (example)
    """
    def __init__(self):
        super().__init__('camera_processor')
        
        # Parameters
        self.declare_parameter('camera_topic', 'image_raw')
        self.declare_parameter('camera_info_topic', 'camera_info')
        self.declare_parameter('publish_processed', True)
        self.declare_parameter('debug', False)
        self.declare_parameter('min_object_size', 100)
        self.declare_parameter('max_objects', 10)
        # Color detection parameters (HSV ranges for red detection)
        self.declare_parameter('red_lower_1', [0, 100, 100])
        self.declare_parameter('red_upper_1', [10, 255, 255])
        self.declare_parameter('red_lower_2', [160, 100, 100])
        self.declare_parameter('red_upper_2', [180, 255, 255])
        
        self.camera_topic = self.get_parameter('camera_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.publish_processed = self.get_parameter('publish_processed').value
        self.debug = self.get_parameter('debug').value
        
        # CV bridge
        self.bridge = CvBridge()
        
        # Camera parameters (will be updated from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Publishers
        self.processed_image_pub = self.create_publisher(
            Image,
            'perception/image_processed',
            qos_profile_sensor_data
        )
        
        self.object_pos_pub = self.create_publisher(
            Point,
            'perception/object_position',
            10
        )
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            qos_profile_sensor_data
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data
        )
        
        self.get_logger().info('Camera processor node started')
    
    def camera_info_callback(self, msg):
        """Store camera calibration parameters."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Received camera calibration parameters')
    
    def image_callback(self, msg):
        """Process incoming image messages."""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Process the image
            processed_image, object_pos = self.process_image(cv_image)
            
            # Publish processed image if enabled
            if self.publish_processed and processed_image is not None:
                try:
                    processed_msg = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
                    processed_msg.header = msg.header
                    self.processed_image_pub.publish(processed_msg)
                except CvBridgeError as e:
                    self.get_logger().error(f'Error converting processed image: {str(e)}')
            
            # Publish object position if detected
            if object_pos is not None:
                point_msg = Point()
                point_msg.x = object_pos[0]  # x position in image
                point_msg.y = object_pos[1]  # y position in image
                point_msg.z = object_pos[2] if len(object_pos) > 2 else 0.0  # depth or size
                self.object_pos_pub.publish(point_msg)
                
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def process_image(self, image):
        """
        Process the input image to detect objects or features.
        
        Args:
            image: Input BGR image (numpy array)
            
        Returns:
            tuple: (processed_image, object_position)
                - processed_image: Image with visualizations (or None if not needed)
                - object_position: (x, y, size) of detected object or None
        """
        # Create a copy of the image for processing
        processed_image = image.copy()
        object_pos = None
        
        # Example: Simple color-based object detection (red objects)
        # Convert to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define range for red color from parameters
        lower_red1 = np.array(self.get_parameter('red_lower_1').value)
        upper_red1 = np.array(self.get_parameter('red_upper_1').value)
        lower_red2 = np.array(self.get_parameter('red_lower_2').value)
        upper_red2 = np.array(self.get_parameter('red_upper_2').value)
        
        # Threshold the HSV image to get only red colors
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Process contours
        if len(contours) > 0:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Get the bounding rectangle
            x, y, w, h = cv2.boundingRect(largest_contour)
            
            # Calculate the center and size
            center_x = x + w // 2
            center_y = y + h // 2
            size = w * h
            
            # Only consider objects above a certain size
            min_size = self.get_parameter('min_object_size').value
            if size > min_size:
                # Draw the bounding box
                cv2.rectangle(processed_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(processed_image, (center_x, center_y), 5, (0, 0, 255), -1)
                
                # Store object position (x, y, size)
                object_pos = (center_x, center_y, size)
                
                if self.debug:
                    self.get_logger().info(f'Detected object at ({center_x}, {center_y}), size: {size}')
        
        return processed_image, object_pos

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CameraProcessor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
