#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import json

class ObjectDetector(Node):
    """
    Object detection node using OpenCV-based detection.
    Can be extended with ML models like YOLO, TensorFlow, etc.
    """
    def __init__(self):
        super().__init__('object_detector')
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('model_path', './yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('nms_threshold', 0.4)
        self.declare_parameter('max_detections', 100)
        self.declare_parameter('debug', False)
        
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.debug = self.get_parameter('debug').value
        
        # Initialize detection method
        self._init_detector()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.detection_pub = self.create_publisher(Image, 'perception/detections', 10)
        self.detection_info_pub = self.create_publisher(String, 'perception/detection_info', 10)
        self.object_count_pub = self.create_publisher(Point, 'perception/object_count', 10)
        
        self.get_logger().info('Object Detector node has been started')
    
    def _init_detector(self):
        """Initialize the detection method."""
        # For now, use OpenCV-based detection
        # This can be replaced with YOLO, TensorFlow, etc.
        self.detection_method = 'opencv'
        
        # Initialize cascade classifiers for face detection (example)
        try:
            self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
            self.get_logger().info('Face detection cascade loaded successfully')
        except Exception as e:
            self.get_logger().warn(f'Could not load face cascade: {e}')
            self.face_cascade = None
    
    def image_callback(self, msg):
        """Process incoming image messages."""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Perform object detection
            detected_image, detections = self.detect_objects(cv_image)
            
            # Publish detection results
            if detected_image is not None:
                detection_msg = self.bridge.cv2_to_imgmsg(detected_image, encoding='bgr8')
                detection_msg.header = msg.header
                self.detection_pub.publish(detection_msg)
            
            # Publish detection information
            if detections:
                detection_info = {
                    'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    'detections': detections,
                    'count': len(detections)
                }
                
                info_msg = String()
                info_msg.data = json.dumps(detection_info)
                self.detection_info_pub.publish(info_msg)
                
                # Publish object count
                count_msg = Point()
                count_msg.x = float(len(detections))
                count_msg.y = 0.0
                count_msg.z = 0.0
                self.object_count_pub.publish(count_msg)
                
                if self.debug:
                    self.get_logger().info(f'Detected {len(detections)} objects')
            
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def detect_objects(self, image):
        """
        Detect objects in the image using OpenCV methods.
        
        Args:
            image: Input BGR image
            
        Returns:
            tuple: (annotated_image, detections_list)
        """
        detections = []
        annotated_image = image.copy()
        
        # Method 1: Face detection using Haar cascades
        if self.face_cascade is not None:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            faces = self.face_cascade.detectMultiScale(
                gray, 
                scaleFactor=1.1, 
                minNeighbors=5, 
                minSize=(30, 30)
            )
            
            for (x, y, w, h) in faces:
                # Draw rectangle around face
                cv2.rectangle(annotated_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                cv2.putText(annotated_image, 'Face', (x, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
                
                detections.append({
                    'class': 'face',
                    'confidence': 0.8,  # Placeholder confidence
                    'bbox': [int(x), int(y), int(w), int(h)],
                    'center': [int(x + w/2), int(y + h/2)]
                })
        
        # Method 2: Simple contour-based detection (for colored objects)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Detect blue objects (example)
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([130, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        
        contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Minimum area threshold
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(annotated_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(annotated_image, 'Blue Object', (x, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                detections.append({
                    'class': 'blue_object',
                    'confidence': 0.7,
                    'bbox': [int(x), int(y), int(w), int(h)],
                    'center': [int(x + w/2), int(y + h/2)],
                    'area': int(area)
                })
        
        return annotated_image, detections

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
