#!/usr/bin/env python3
"""
Dependency verification script for robot_perception package.
This script tests all the imports used in the perception nodes.
"""

def test_camera_processor_imports():
    """Test imports for camera_processor.py"""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image, CameraInfo
        from cv_bridge import CvBridge, CvBridgeError
        import cv2
        import numpy as np
        from geometry_msgs.msg import Point
        print("✓ camera_processor imports successful")
        return True
    except ImportError as e:
        print(f"✗ camera_processor import error: {e}")
        return False

def test_object_detector_imports():
    """Test imports for object_detector.py"""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        from geometry_msgs.msg import Point
        from cv_bridge import CvBridge, CvBridgeError
        import cv2
        import numpy as np
        import json
        print("✓ object_detector imports successful")
        return True
    except ImportError as e:
        print(f"✗ object_detector import error: {e}")
        return False

def test_package_imports():
    """Test package-level imports"""
    try:
        from robot_perception import CameraProcessor, ObjectDetector
        print("✓ package imports successful")
        return True
    except ImportError as e:
        print(f"✗ package import error: {e}")
        return False

def main():
    """Run all import tests"""
    print("Testing robot_perception dependencies...")
    
    success = True
    success &= test_camera_processor_imports()
    success &= test_object_detector_imports()
    success &= test_package_imports()
    
    if success:
        print("\n✓ All dependency tests passed!")
    else:
        print("\n✗ Some dependency tests failed!")
    
    return success

if __name__ == "__main__":
    main()