# Robot Car (ROS 2 Humble) Workspace

A comprehensive ROS 2 Humble workspace for a Raspberry Pi 4 robot car with Arduino integration, real-time vision processing, and autonomous navigation capabilities.

## Key Features

- **Arduino Integration**: Bidirectional communication between ROS 2 and Arduino for motor control and sensor reading
- **Real-time Vision**: YOLOv8-based object detection with hardware acceleration
- **Sensor Suite**: Support for RPi Camera, LiDAR, and various analog/digital sensors
- **Navigation**: Integration with Nav2 for autonomous navigation
- **Modular Design**: Clean separation of concerns with dedicated packages for each component
- **Optimized Performance**: Model quantization and layer freezing for edge deployment

## Hardware Requirements

- Raspberry Pi 4 (4GB+ recommended)
- Arduino board (Uno, Mega, or similar)
- Motor controller (compatible with Arduino)
- RPi Camera Module
- LiDAR sensor (e.g., RPLIDAR)
- Power supply (battery pack)
- USB connections between Pi and Arduino

## Software Dependencies

### Core Dependencies
```bash
sudo apt update
sudo apt install -y \
  python3-pip python3-opencv python3-serial \
  ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-camera-info-manager ros-humble-rclpy \
  ros-humble-sensor-msgs ros-humble-std-msgs \
  ros-humble-nav2-bringup ros-humble-nav2-msgs
```

## Workspace Structure

```
Dojo/
├── src/
│   ├── arduino_bridge/       # ROS 2 - Arduino communication
│   │   ├── arduino_bridge/   # Python package
│   │   ├── launch/          # Launch files
│   │   ├── config/          # Configuration files
│   │   └── resource/        # Package resources
│   │
│   ├── vision_system/        # Real-time vision processing
│   │   ├── vision_system/    # Python package
│   │   │   └── object_detector.py  # Main detection node
│   │   ├── launch/          # Launch configurations
│   │   ├── config/          # YAML parameter files
│   │   └── models/          # ML models (YOLO, etc.)
│   │
│   ├── robot_bringup/       # System launch files
│   ├── robot_control/       # Motor and actuator control
│   ├── robot_sensors/       # Sensor drivers
│   ├── robot_perception/    # High-level perception
│   ├── robot_description/   # URDF/Xacro models
│   ├── robot_navigation/    # Navigation stack
│   └── ros2arduino_bridge/  # Alternative Arduino bridge
│
├── build/                   # Build files
├── install/                 # Installed packages
└── log/                    # Log files
```

### Key Packages

#### `arduino_bridge`
- Bidirectional ROS 2 - Arduino communication
- Handles motor control and sensor data
- Configurable serial interface

#### `vision_system` (New!)
- Real-time object detection with YOLOv8
- Optimized for edge devices (RPi 4)
- Features:
  - Model quantization (FP16/INT8)
  - Layer freezing for efficient training
  - Mosaic augmentation
  - TensorRT acceleration

#### `robot_control`
- Motor control interfaces
- Low-level actuator commands
- Safety features

#### `robot_sensors`
- Camera drivers
- LiDAR integration
- IMU and other sensor interfaces

#### `robot_perception`
- High-level perception tasks
- Sensor fusion
- Object tracking

#### `robot_navigation`
- Navigation stack integration
- SLAM configuration
- Path planning

## Getting Started

### 1. Install Dependencies
```bash
# Core ROS 2 and Python dependencies
sudo apt update
sudo apt install -y \
  python3-pip python3-opencv python3-serial \
  ros-humble-cv-bridge ros-humble-image-transport \
  ros-humble-camera-info-manager ros-humble-rclpy \
  ros-humble-sensor-msgs ros-humble-std-msgs \
  ros-humble-nav2-bringup ros-humble-nav2-msgs

# Python packages
pip3 install ultralytics onnxruntime opencv-python torch torchvision
```

### 2. Build the Workspace
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build all packages
cd ~/Dojo
colcon build --symlink-install
source install/setup.bash
```

### 3. Run the Vision System
```bash
# Start the vision system
ros2 launch vision_system vision_system.launch.py

# In a new terminal, view detection results
ros2 topic echo /detections
```

### 4. Run with Arduino Bridge
```bash
# Terminal 1: Start the vision system
ros2 launch vision_system vision_system.launch.py

# Terminal 2: Start the Arduino bridge
ros2 run arduino_bridge arduino_bridge_node
```

## Arduino Communication Protocol

The Arduino bridge uses a simple text-based protocol over serial:

### From Arduino to ROS 2
- Sensor data: `S<value>\n`
  - Example: `S512\n` sends sensor value 512

### From ROS 2 to Arduino
- LED control: `L<value>\n`
  - Example: `L1\n` turns LED on, `L0\n` turns it off

## Customization

### Adding New Sensors
1. Modify the Arduino sketch to read the sensor
2. Update the protocol in both Arduino and ROS 2 code
3. Add new message types if needed

### Changing Serial Port
```bash
ros2 run arduino_bridge arduino_bridge_node --ros-args -p port:=/dev/ttyUSB0
```

## Troubleshooting

### Common Issues

#### 1. Camera Not Detected
```bash
# Check video devices
ls /dev/video*

# Check video group membership
groups | grep video

# Add user to video group if needed
sudo usermod -a -G video $USER
```

#### 2. Serial Port Permissions
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Check connected devices
ls -l /dev/ttyACM* /dev/ttyUSB*
```

#### 3. Model Loading Issues
- Ensure model files are in the correct location (`vision_system/models/`)
- Check file permissions
- Verify model compatibility with your hardware

#### 4. Performance Optimization
- Reduce input resolution in `object_detector_params.yaml`
- Enable FP16/INT8 quantization
- Close unnecessary applications to free up memory

#### 5. ROS 2 Node Issues
```bash
# Source the workspace
source install/setup.bash

# List all running nodes
ros2 node list

# View node output
ros2 topic echo /detections
```

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please read our [contributing guidelines](CONTRIBUTING.md) before submitting pull requests.
