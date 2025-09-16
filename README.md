# Dojo - ROS 2 Robot Platform

A modular ROS 2 Humble workspace for an autonomous robot with sensor integration, computer vision, and navigation capabilities.

## 🚀 Key Features

- **Modular Architecture**: Organized into separate packages for sensors, control, and perception
- **Sensor Integration**:
  - RPLIDAR A1M8 for 360° laser scanning
  - Camera with libcamera support for vision tasks
- **Robust Control**:
  - Differential drive control system
  - Arduino-based motor control
  - Configurable PID parameters
- **Perception Stack**:
  - Object detection pipeline
  - Sensor fusion capabilities
- **Navigation**:
  - Integration with Nav2
  - TF2 for coordinate transforms
  - Configurable launch files

## 🛠️ Hardware Requirements

- **Compute**: Raspberry Pi 4 (4GB+ recommended) or x86_64 PC
- **Motors**:
  - 2x DC motors with encoders
  - Motor driver (e.g., L298N, TB6612FNG)
- **Sensors**:
  - RPLIDAR A1M8
  - Raspberry Pi Camera or USB webcam
- **Controller**: Arduino (Uno/Mega) for low-level motor control
- **Power**: 12V battery pack with appropriate voltage regulation

## 📦 Package Structure

```
Dojo/
├── src/
│   ├── robot_bringup/         # Launch files and configurations
│   ├── robot_control/         # Motor control and Arduino bridge
│   │   ├── nodes/
│   │   │   ├── arduino_bridge.py
│   │   │   └── cmd_vel_to_motors.py
│   │   └── launch/
│   │       └── control.launch.py
│   │
│   ├── robot_sensors/         # Sensor interfaces
│   │   ├── nodes/
│   │   │   └── camera_node.py
│   │   └── launch/
│   │       ├── sensors.launch.py
│   │       ├── camera.launch.py
│   │       └── rplidar.launch.py
│   │
│   ├── robot_description/     # Robot URDF and meshes
│   │   └── urdf/
│   │       ├── robot.urdf.xacro
│   │       └── sensors/
│   │           └── rplidar.urdf.xacro
│   │
│   └── robot_perception/      # Vision and perception
│       └── nodes/
│           └── object_detector.py
```

## 🛠️ Installation

### 1. System Dependencies

```bash
# Install ROS 2 Humble (if not already installed)
sudo apt update && sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# Install ROS 2 packages
sudo apt install -y \
    ros-humble-rplidar-ros \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-tf2-ros \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox
```

### 2. Workspace Setup

```bash
# Create workspace
mkdir -p ~/Dojo/src
cd ~/Dojo

# Clone the repository (or copy your code)
git clone <repository-url> src

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the workspace
echo "source ~/Dojo/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 🚀 Getting Started

### Launching the Robot

#### Full System Startup
```bash
# Source the workspace
source install/setup.bash

# Launch the complete system
ros2 launch robot_bringup bringup.launch.py
```

#### Individual Components

**Start Sensors Only**
```bash
ros2 launch robot_sensors sensors.launch.py
```

**Start Control System**
```bash
ros2 launch robot_control control.launch.py
```

**Visualize in RViz**
```bash
rviz2 -d src/robot_bringup/rviz/robot.rviz
```

## 🎯 Key Topics

- **Command Velocity**: `/cmd_vel` (Twist)
- **Camera Feed**: `/camera/image_raw` (Image)
- **LiDAR Data**: `/scan` (LaserScan)
- **Motor Commands**: `/left_motor/speed`, `/right_motor/speed` (Float32)

## ⚙️ Configuration

### RPLIDAR Parameters
Edit `robot_sensors/launch/rplidar.launch.py` to adjust:
- Serial port (`/dev/ttyUSB0` by default)
- Frame ID (`laser` by default)
- Scan frequency (10Hz by default)

### Camera Parameters
Edit `robot_sensors/launch/camera.launch.py` to configure:
- Resolution (default: 640x480)
- Frame rate (default: 30fps)
- Frame ID (`camera_link` by default)

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- ROS 2 Community
- RPLIDAR Team
- OpenCV Community
- All contributors and maintainers
