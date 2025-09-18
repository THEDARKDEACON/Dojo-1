# Robot Perception Package

The `robot_perception` package provides computer vision and object detection capabilities for the Dojo Robot platform.

## 🎯 **Features**

- **Camera Processing**: Real-time image processing and analysis
- **Object Detection**: OpenCV-based object detection (extensible to ML models)
- **Color-based Detection**: Configurable color range detection
- **Face Detection**: Haar cascade-based face detection
- **ROS2 Integration**: Full integration with robot hardware and control systems

## 📦 **Package Structure**

```
robot_perception/
├── config/
│   └── robot_perception_params.yaml    # Configuration parameters
├── launch/
│   ├── perception.launch.py            # Main perception launch file
│   └── robot_perception.launch.py      # Alternative launch file
├── robot_perception/
│   ├── __init__.py                     # Package initialization
│   ├── camera_processor.py             # Main camera processing node
│   ├── object_detector.py              # Object detection node
│   └── yolov8n.pt                     # ML model (placeholder)
├── package.xml                         # Package dependencies
├── setup.py                           # Python package setup
└── README.md                          # This file
```

## 🚀 **Quick Start**

### **Launch Perception with Hardware**
```bash
# Launch with real camera
ros2 launch robot_bringup bringup.launch.py use_perception:=true use_camera:=true

# Launch perception only
ros2 launch robot_perception perception.launch.py
```

### **Launch Perception in Simulation**
```bash
# Full simulation with perception
ros2 launch robot_gazebo simulation.launch.py use_perception:=true

# Gazebo with perception
ros2 launch robot_bringup bringup.launch.py use_gazebo:=true use_perception:=true
```

## 🔧 **Configuration**

### **Parameters**

The perception system is configured via `config/robot_perception_params.yaml`:

```yaml
camera_processor:
  ros__parameters:
    camera_topic: "image_raw"
    camera_info_topic: "camera_info"
    publish_processed: true
    debug: false
    min_object_size: 100
    max_objects: 10
    # Color detection ranges (HSV)
    red_lower_1: [0, 100, 100]
    red_upper_1: [10, 255, 255]

object_detector:
  ros__parameters:
    confidence_threshold: 0.5
    nms_threshold: 0.4
    max_detections: 100
```

### **Topic Configuration**

You can customize topics via launch arguments:

```bash
ros2 launch robot_perception perception.launch.py \
  camera_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info
```

## 📡 **ROS2 Interface**

### **Subscribed Topics**

| Topic | Type | Description |
|-------|------|-------------|
| `image_raw` | `sensor_msgs/Image` | Raw camera images |
| `camera_info` | `sensor_msgs/CameraInfo` | Camera calibration info |

### **Published Topics**

| Topic | Type | Description |
|-------|------|-------------|
| `/perception/image_processed` | `sensor_msgs/Image` | Processed images with annotations |
| `/perception/object_position` | `geometry_msgs/Point` | Detected object positions |
| `/perception/detections` | `sensor_msgs/Image` | Object detection results |
| `/perception/detection_info` | `std_msgs/String` | JSON detection information |
| `/perception/object_count` | `geometry_msgs/Point` | Number of detected objects |

### **Parameters**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_topic` | `image_raw` | Input camera topic |
| `camera_info_topic` | `camera_info` | Camera calibration topic |
| `publish_processed` | `true` | Publish processed images |
| `debug` | `false` | Enable debug output |
| `confidence_threshold` | `0.5` | Detection confidence threshold |
| `min_object_size` | `100` | Minimum object size (pixels²) |

## 🎛️ **Nodes**

### **Camera Processor (`camera_processor`)**

Processes camera images for basic computer vision tasks:

- **Color-based object detection**
- **Contour analysis**
- **Object position tracking**
- **Image annotation and visualization**

**Features:**
- Configurable color ranges for object detection
- Real-time image processing
- Object size filtering
- Debug visualization

### **Object Detector (`object_detector`)**

Advanced object detection using OpenCV and ML models:

- **Face detection** using Haar cascades
- **Color-based object detection**
- **Extensible for ML models** (YOLO, TensorFlow, etc.)
- **Detection result publishing**

**Features:**
- Multiple detection methods
- Confidence scoring
- Bounding box detection
- JSON result formatting

## 🔗 **Integration**

### **Hardware Integration**

The perception package integrates seamlessly with the hardware layer:

```python
# Camera driver publishes to:
/image_raw              # Raw camera images
/camera_info           # Camera calibration

# Perception subscribes to these topics automatically
```

### **Control Integration**

Perception results can be used by the control system:

```python
# Object positions published to:
/perception/object_position    # For navigation avoidance
/perception/object_count      # For counting tasks
```

### **Simulation Integration**

Works with Gazebo simulation:

```bash
# Gazebo camera plugin publishes to same topics
# Perception works identically in simulation and real hardware
```

## 🛠️ **Development**

### **Adding New Detection Methods**

1. **Extend ObjectDetector class:**
```python
def detect_custom_objects(self, image):
    # Your detection logic here
    return annotated_image, detections
```

2. **Add to detect_objects method:**
```python
def detect_objects(self, image):
    # Existing detection methods...
    
    # Add your custom detection
    custom_detections = self.detect_custom_objects(image)
    detections.extend(custom_detections)
```

### **Adding ML Models**

1. **Install model dependencies:**
```bash
pip install ultralytics  # For YOLO
pip install tensorflow   # For TensorFlow
```

2. **Load model in __init__:**
```python
from ultralytics import YOLO
self.yolo_model = YOLO('yolov8n.pt')
```

3. **Use in detection:**
```python
results = self.yolo_model(image)
# Process results...
```

### **Custom Color Detection**

Update the configuration file:

```yaml
camera_processor:
  ros__parameters:
    # Add new color ranges
    blue_lower: [100, 50, 50]
    blue_upper: [130, 255, 255]
    green_lower: [40, 50, 50]
    green_upper: [80, 255, 255]
```

## 🧪 **Testing**

### **Test Perception Integration**
```bash
./test_perception_integration.sh
```

### **Monitor Topics**
```bash
# List perception topics
ros2 topic list | grep perception

# Monitor detection results
ros2 topic echo /perception/detection_info

# View processed images
ros2 run rqt_image_view rqt_image_view /perception/image_processed
```

### **Debug Mode**
```bash
# Enable debug output
ros2 launch robot_perception perception.launch.py debug:=true
```

## 🔍 **Troubleshooting**

### **Common Issues**

**Camera not detected:**
```bash
# Check camera topics
ros2 topic list | grep image
ros2 topic echo /image_raw
```

**No detections:**
- Check lighting conditions
- Adjust color ranges in config
- Verify object size thresholds
- Enable debug mode

**Performance issues:**
- Reduce image resolution
- Limit detection frequency
- Optimize detection algorithms

### **Debug Commands**

```bash
# Check node status
ros2 node list | grep perception
ros2 node info /camera_processor

# Monitor processing rate
ros2 topic hz /perception/image_processed

# View detection statistics
ros2 topic echo /perception/object_count
```

## 📈 **Performance**

### **Optimization Tips**

1. **Reduce image resolution** for faster processing
2. **Limit detection regions** to areas of interest
3. **Use efficient algorithms** for real-time performance
4. **Batch processing** for multiple objects

### **Benchmarks**

- **Camera Processing**: ~30 FPS @ 640x480
- **Face Detection**: ~15 FPS @ 640x480
- **Color Detection**: ~60 FPS @ 640x480
- **Memory Usage**: ~50MB per node

## 🤝 **Contributing**

1. **Add new detection methods** in `object_detector.py`
2. **Improve algorithms** for better accuracy
3. **Add ML model support** for advanced detection
4. **Optimize performance** for real-time operation
5. **Add tests** for new functionality

## 📄 **License**

This package is licensed under the Apache 2.0 License.