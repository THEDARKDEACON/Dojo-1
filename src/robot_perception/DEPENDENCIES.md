# Robot Perception Dependencies

This document lists all dependencies required for the robot_perception package.

## ROS2 Dependencies (package.xml)

### Core ROS2 packages:
- `rclpy` - ROS2 Python client library
- `sensor_msgs` - Sensor message types (Image, CameraInfo)
- `std_msgs` - Standard message types (String)
- `geometry_msgs` - Geometry message types (Point)
- `builtin_interfaces` - Built-in ROS2 interfaces

### Computer Vision packages:
- `cv_bridge` - Bridge between ROS and OpenCV
- `image_transport` - Image transport mechanisms
- `vision_opencv` - OpenCV integration for ROS

### System packages:
- `python3-opencv` - OpenCV Python bindings
- `python3-numpy` - NumPy for numerical operations

## Python Dependencies (setup.py)

### Required packages:
- `setuptools` - Python package setup tools
- `opencv-python` - OpenCV Python bindings
- `numpy` - Numerical computing library

### Built-in modules used:
- `json` - JSON encoding/decoding (built into Python)

## Import Verification

The following imports are used in the perception nodes:

### camera_processor.py:
```python
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Point
```

### object_detector.py:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import json
```

## Testing Dependencies

To test if all dependencies are available, run:
```bash
python3 src/robot_perception/test_dependencies.py
```

## Notes

- OpenCV Haar cascades are loaded dynamically and will gracefully fail if not available
- All message types are standard ROS2 messages
- The package uses only stable, well-supported dependencies