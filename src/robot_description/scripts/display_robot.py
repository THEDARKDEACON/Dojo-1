#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os

class DisplayRobot(Node):
    def __init__(self):
        super().__init__('display_robot')
        self.get_logger().info('Display Robot Node Started')

def main(args=None):
    rclpy.init(args=args)
    node = DisplayRobot()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
