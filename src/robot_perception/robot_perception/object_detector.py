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
            cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            self.face_cascade = cv2.CascadeClassifier(cascade_path)
            # Verify the cascade loaded correctly
            if self.face_cascade.empty():
                raise Exception(f"Failed to load cascade from {cascade_path}")
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
        
        # Method 2: Edge-based object detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Edge detection using Canny
        edges = cv2.Canny(blurred, 50, 150)
        
        # Find contours from edges
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:  # Minimum area threshold
                # Get bounding rectangle
                x, y, w, h = cv2.boundingRect(contour)
                
                # Calculate aspect ratio to classify object shape
                aspect_ratio = float(w) / h
                
                # Classify based on shape characteristics
                if 0.8 <= aspect_ratio <= 1.2:
                    object_type = "Square/Circle"
                    color = (0, 255, 0)  # Green
                elif aspect_ratio > 2.0:
                    object_type = "Horizontal Rectangle"
                    color = (255, 0, 0)  # Blue
                elif aspect_ratio < 0.5:
                    object_type = "Vertical Rectangle"
                    color = (0, 0, 255)  # Red
                else:
                    object_type = "Rectangle"
                    color = (255, 255, 0)  # Cyan
                
                # Draw bounding box and label
                cv2.rectangle(annotated_image, (x, y), (x + w, y + h), color, 2)
                cv2.putText(annotated_image, object_type, (x, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                
                detections.append({
                    'class': object_type.lower().replace('/', '_'),
                    'confidence': 0.8,
                    'bbox': [int(x), int(y), int(w), int(h)],
                    'center': [int(x + w/2), int(y + h/2)],
                    'area': int(area),
                    'aspect_ratio': round(aspect_ratio, 2)
                })
        
        # Method 3: Blob detection for circular objects
        # Set up SimpleBlobDetector parameters
        params = cv2.SimpleBlobDetector_Params()
        
        # Filter by Area
        params.filterByArea = True
        params.minArea = 500
        params.maxArea = 50000
        
        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.3
        
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.5
        
        # Create detector
        detector = cv2.SimpleBlobDetector_create(params)
        
        # Detect blobs
        keypoints = detector.detect(blurred)
        
        # Draw detected blobs
        for keypoint in keypoints:
            x, y = int(keypoint.pt[0]), int(keypoint.pt[1])
            radius = int(keypoint.size / 2)
            
            # Draw circle around blob
            cv2.circle(annotated_image, (x, y), radius, (255, 0, 255), 2)  # Magenta
            cv2.putText(annotated_image, 'Blob', (x - 20, y - radius - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
            
            detections.append({
                'class': 'blob',
                'confidence': 0.9,
                'bbox': [int(x - radius), int(y - radius), int(2 * radius), int(2 * radius)],
                'center': [x, y],
                'radius': radius,
                'area': int(3.14159 * radius * radius)
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
