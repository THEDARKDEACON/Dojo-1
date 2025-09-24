#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class Nv21Converter(Node):

    def __init__(self):
        super().__init__('nv21_converter')
        self.publisher = self.create_publisher(Image, '/camera/image_bgr8', 10)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info('NV21 converter node started, waiting for images...')

    def image_callback(self, msg):
        try:
            # Manually convert the ROS Image message data to a NumPy array
            # The data is a flattened byte array.
            data = np.frombuffer(msg.data, dtype=np.uint8)

            # Reshape the data to the correct image dimensions for NV21
            # NV21 has Y plane (height*width) and a shared UV plane (height*width/2)
            # The total height for the flattened array is height * 1.5
            height = msg.height
            width = msg.width
            cv_image = data.reshape((int(height * 1.5), width))

            # Convert the NV21 color space to BGR8
            bgr8_image = cv2.cvtColor(cv_image, cv2.COLOR_YUV2BGR_NV21)
            
            # Convert the OpenCV Mat object back to a ROS Image message
            ros_image_msg = self.bridge.cv2_to_imgmsg(bgr8_image, encoding='bgr8')
            
            # Set the header to match the original message
            ros_image_msg.header = msg.header
            
            # Publish the new BGR8 image message
            self.publisher.publish(ros_image_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error during image conversion: {e}")

def main(args=None):
    rclpy.init(args=args)
    nv21_converter = Nv21Converter()
    rclpy.spin(nv21_converter)
    nv21_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
