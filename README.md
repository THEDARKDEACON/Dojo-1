# Dojo Robot - ROS2 Robotics Platform

A modular, safety-focused ROS2 robotics platform for autonomous navigation and manipulation, featuring advanced command multiplexing with `twist_mux` and seamless simulation integration.

## 🚀 Quick Start

### Prerequisites
- ROS 2 Humble (or newer)
- Python 3.8+
- Arduino IDE (for firmware uploads)
- Gazebo Fortress (for simulation)
  ```bash
  sudo apt install ros-${ROS_DISTRO}-gazebo-ros-pkgs
  ```

### Prerequisites
- ROS 2 Humble (or newer)
- Python 3.8+
- Arduino IDE (for firmware uploads)
- Gazebo Fortress (for simulation)

### Building the Workspace
```bash
# Build all packages
colcon build --symlink-install
source install/setup.bash

# Build specific package
# colcon build --packages-select <package_name>

# Build with warnings as errors (recommended)
# colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-Wall -Wextra -Wpedantic"
```

### Launching the Robot

#### Real Robot (Hardware)
```bash
# Minimal setup (Arduino only)
ros2 launch robot_launch robot_complete.launch.py use_sim_time:=false

# With all sensors
ros2 launch robot_launch robot_complete.launch.py use_sim_time:=false \
  use_camera:=true use_lidar:=true
```

#### Simulation (Gazebo)
```bash
# Start Gazebo simulation
ros2 launch robot_gazebo robot_simulation.launch.py

# With RViz
ros2 launch robot_gazebo robot_simulation.launch.py use_rviz:=true
```

#### Teleoperation
```bash
# Keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_teleop

# Joystick (if configured)
ros2 launch teleop_twist_joy teleop-launch.py
```

## 🛠️ Recent Updates

### Latest Improvements

1. **twist_mux Integration**
   - Added priority-based command arbitration
   - Configurable timeouts and priorities
   - Seamless switching between control modes

2. **Simulation Enhancements**
   - Gazebo integration with twist_mux
   - RViz visualization tools
   - Parameterized launch configurations

3. **Safety Features**
   - Emergency stop handling
   - Command validation
   - Hardware health monitoring

### Getting the Latest Version
```bash
git pull origin main
colcon build --symlink-install
source install/setup.bash
```

## 🏗️ System Architecture

The Dojo robot features a robust, safety-focused architecture with command multiplexing and hardware abstraction:

```
┌─────────────────────────────────────────────────────────────┐
│                    NAVIGATION LAYER                         │
│              (Path Planning, SLAM, Mapping)                 │
├─────────────────────────────────────────────────────────────┤
│                    PERCEPTION LAYER                         │
│            (Computer Vision, Object Detection)              │
├─────────────────────────────────────────────────────────────┤
│                     CONTROL LAYER                           │
│  (twist_mux, Safety Systems, Command Filtering, Modes)      │
│  ┌──────────────────────────────────────────────────────┐   │
│  │                  twist_mux (200)                    │   │
│  │  ┌──────────────┐  ┌──────────────┐  ┌───────────┐  │   │
│  │  │  Safety      │  │  Teleop      │  │ Navigation│  │   │
│  │  │  (Highest)   │  │  (Medium)    │  │ (Lowest)  │  │   │
│  └──┴──────┬───────┴──┴──────┬───────┴──┴────┬──────┘  │
└────────────┼─────────────────┼────────────────┼─────────┘
             │                 │                │
┌────────────▼─────────────────▼────────────────▼────────────┐
│                    HARDWARE ABSTRACTION                    │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────┐  │
│  │ Motor Driver │  │ Camera       │  │ LiDAR          │  │
│  │ (Arduino)    │  │ (ROS2 Driver)│  │ (RPLIDAR)      │  │
│  └──────────────┘  └──────────────┘  └─────────────────┘  │
└────────────────────────────────────────────────────────────┘
```

### Key Features

- **Command Velocity Multiplexing**: Priority-based command arbitration via `twist_mux`
  - Safety commands (highest priority, 200)
  - Teleoperation commands (medium priority, 100)
  - Autonomous navigation (lowest priority, 10)

- **Simulation Support**:
  - Full Gazebo integration with hardware-in-the-loop testing
  - RViz visualization tools
  - Parameterized launch files for easy switching between sim/hardware

- **Hardware Abstraction**:
  - Clean separation between hardware and control logic
  - Support for multiple sensor configurations
  - Easy hardware interface expansion

- **Safety Features**:
  - Emergency stop handling
  - Command validation and filtering
  - Hardware health monitoring
  - Graceful degradation

**🛡️ Safety First**: Multiple safety layers prevent accidents
- Emergency stops at hardware and control levels
- Obstacle detection and avoidance
- Velocity limiting and command filtering
- Hardware health monitoring

**🔧 Reliability**: Robust error handling and recovery
- Auto-reconnection for all hardware
- Graceful degradation when components fail
- Real-time status monitoring and diagnostics

**📈 Scalability**: Easy to extend and modify
- Modular design - add new sensors without breaking existing code
- Standardized interfaces between layers
- Centralized configuration management

## 📦 Package Structure

### Core Packages

| Package | Purpose | Key Features |
|---------|---------|--------------|
| `robot_hardware` | **Unified Hardware Interface** | Arduino, camera, LiDAR drivers + health monitoring |
| `robot_control` | **High-Level Control** | Safety systems, command filtering, mode management |
| `robot_interfaces` | **Custom Messages** | Standardized data structures and services |
| `robot_bringup` | **System Orchestration** | Launch files for complete system startup |

### Optional Packages

| Package | Purpose | Status |
|---------|---------|--------|
| `robot_perception` | Computer vision, object detection | Optional |
| `robot_navigation` | Autonomous navigation, SLAM | Optional |
| `robot_description` | URDF models, visualization | Available |

### Legacy Packages (Being Phased Out)

| Package | Replacement | Status |
|---------|-------------|--------|
| `arduino_bridge` | `robot_hardware` | ⚠️ Deprecated |
| `ros2arduino_bridge` | `robot_hardware` | ⚠️ Deprecated |
| `robot_sensors` | `robot_hardware` | ⚠️ Deprecated |

## 🔌 Hardware Components

### Supported Hardware

- **Arduino Uno/Nano** - Motor control, encoders, sensors
- **USB Camera** - Computer vision, streaming
- **RPLiDAR A1/A2** - 360° laser scanning
- **Ultrasonic Sensors** - Obstacle detection
- **IMU** (optional) - Orientation sensing

### Hardware Connections

```
Raspberry Pi 4
├── USB0: Arduino (/dev/ttyACM0)
├── USB1: LiDAR (/dev/ttyUSB0)  
├── USB2: Camera (/dev/video0)
└── GPIO: Additional sensors
```

## ⚙️ Configuration

All hardware settings are centralized in `src/robot_hardware/config/hardware.yaml`:

```yaml
## 🛠️ Configuration

### Hardware Interface

#### Arduino Driver (`arduino_driver`)
```yaml
arduino_driver:
  ros__parameters:
    port: "/dev/ttyACM0"
    baud_rate: 115200
    wheel_base: 0.2
    wheel_radius: 0.05
    max_rpm: 60
    publish_rate: 50  # Hz
```

#### twist_mux Configuration
```yaml
twist_mux:
  ros__parameters:
    # Safety (highest priority)
    - name: safety
      topic: cmd_vel_safety
      timeout: 0.5
      priority: 200
    
    # Teleoperation (medium priority)
    - name: teleop
      topic: cmd_vel_teleop
      timeout: 1.0
      priority: 100
    
    # Navigation (lowest priority)
    - name: nav
      topic: cmd_vel_nav
      timeout: 1.0
      priority: 10
```

#### PID Controller
```yaml
diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.3
    wheel_radius: 0.05
    cmd_vel_topic: cmd_vel
    odom_frame_id: odom
    base_frame_id: base_footprint
```
    wheel_radius: 0.033
    max_speed: 0.5

camera_driver:
  ros__parameters:
    width: 640
    height: 480
    fps: 30.0

lidar_driver:
  ros__parameters:
    serial_port: "/dev/ttyUSB0"
    frame_id: "laser"
```

## 🎮 Usage Examples

### Basic Robot Control

```bash
# Start the robot
ros2 launch robot_bringup bringup.launch.py

# Control manually with keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Check system status
ros2 topic echo /system_status
ros2 topic echo /diagnostics
```

### Simulation vs Real Hardware

- Real hardware (no Gazebo): set `use_gazebo:=false` so the URDF doesn’t require simulation-only dependencies. Example:
  - `ros2 launch robot_bringup bringup.launch.py use_gazebo:=false`
- Simulation (Gazebo): set `use_gazebo:=true use_sim_time:=true` and ensure the `robot_gazebo/` package is built and sourced.
  - `colcon build --packages-select robot_gazebo && source install/setup.bash`
  - `ros2 launch robot_bringup bringup.launch.py use_gazebo:=true use_sim_time:=true`
- Sensor toggles: `use_arduino`, `use_camera`, `use_lidar` are available in both `bringup.launch.py` and `hardware.launch.py` to selectively enable drivers.

### Hardware Testing

```bash
# Test individual components
ros2 launch robot_hardware hardware.launch.py use_arduino:=true use_camera:=false use_lidar:=false

# Monitor hardware health
ros2 topic echo /arduino_status
ros2 topic echo /camera_status
ros2 topic echo /lidar_status
```

### Safety Features

```bash
# Emergency stop
ros2 topic pub /emergency_stop_request std_msgs/Bool "data: true"

# Check safety status
ros2 topic echo /control_status
ros2 topic echo /emergency_stop
```
## 🛠️ 
Development Guide

### Adding New Hardware

1. **Create driver in `robot_hardware/drivers/`**:
```python
class NewSensorDriver(Node):
    def __init__(self):
        super().__init__('new_sensor_driver')
        # Your sensor code here
```

2. **Add to hardware manager**:
```python
# In hardware_manager.py
self.new_sensor_status_sub = self.create_subscription(
    String, 'new_sensor_status', self._new_sensor_callback, 10)
```

3. **Update configuration**:
```yaml
# In hardware.yaml
new_sensor_driver:
  ros__parameters:
    port: "/dev/ttyUSB1"
    # Your parameters here
```

4. **Add to launch file**:
```python
# In hardware.launch.py
new_sensor_node = Node(
    package='robot_hardware',
    executable='new_sensor_driver',
    # ...
)
```

### Safety System Integration

All new components should integrate with the safety system:

```python
# Publish status for monitoring
self.status_pub = self.create_publisher(String, 'component_status', 10)

# Listen for emergency stops
self.estop_sub = self.create_subscription(
    Bool, 'emergency_stop', self._estop_callback, 10)

def _estop_callback(self, msg):
    if msg.data:
        self.stop_all_operations()
```

## 🐛 Common Issues & Solutions

### twist_mux Not Working
```bash
# Check if twist_mux is running
ros2 node list | grep twist_mux

# Check active topics
ros2 topic list | grep cmd_vel

# View twist_mux status
ros2 topic echo /twist_mux/status
```

### Simulation Issues
```bash
# Check Gazebo model paths
echo $GAZEBO_MODEL_PATH

# Run Gazebo in verbose mode
ros2 launch robot_gazebo robot_simulation.launch.py verbose:=true
```

### Hardware Connection Problems
```bash
# Check USB devices
lsusb

# Check device permissions
ls -l /dev/tty*

# Set correct permissions
sudo usermod -a -G dialout $USER
```

## 📚 Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/)
- [Gazebo Tutorials](http://gazebosim.org/tutorials)
- [twist_mux Documentation](https://github.com/ros2/teleop_twist_joy)
- [Arduino-ROS2 Integration](https://github.com/ROBOTIS-GIT/ros2arduino_bridge)

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📜 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

### Common Issues

**Arduino not connecting:**
```bash
# Check USB connection
ls /dev/tty*
# Should see /dev/ttyACM0 or similar

# Check permissions
sudo usermod -a -G dialout $USER
# Logout and login again
```

**Camera not working:**
```bash
# Check camera device
ls /dev/video*
# Should see /dev/video0

# Test camera directly
v4l2-ctl --list-devices
```

**LiDAR not spinning:**
```bash
# Check USB connection
ls /dev/ttyUSB*
# Should see /dev/ttyUSB0

# Check power supply (LiDAR needs 5V)
```

### Debug Commands

```bash
# Check all ROS topics
ros2 topic list

# Monitor system health
ros2 topic echo /diagnostics

# Check node status
ros2 node list
ros2 node info /hardware_manager

# View logs
ros2 log view
```

### Performance Monitoring

```bash
# CPU and memory usage
htop

# ROS2 performance
ros2 run rqt_graph rqt_graph
ros2 run rqt_plot rqt_plot

# Network bandwidth
iftop
```

## 🔧 Build System

### Dependencies

**System packages:**
```bash
sudo apt install ros-humble-desktop python3-colcon-common-extensions
sudo apt install python3-opencv python3-serial python3-numpy
```

**ROS2 packages:**
```bash
sudo apt install ros-humble-tf2-ros ros-humble-geometry-msgs
sudo apt install ros-humble-sensor-msgs ros-humble-nav-msgs
```

### Build Process

The build system automatically:
1. **Validates dependencies** - Checks all required packages
2. **Handles package order** - Builds in correct dependency order  
3. **Manages Python paths** - Sets up module imports correctly
4. **Installs configurations** - Copies config files to install space

```bash
# Full build (recommended)
./build_ros2_pi.sh

# Individual package build
colcon build --packages-select robot_hardware

# Clean build
rm -rf build/ install/ log/
./build_ros2_pi.sh
```

#### Notes

- To avoid legacy package issues (e.g., `ros2arduino_bridge` installing to `/lib`), skip legacy packages during build:
  - `colcon build --packages-skip ros2arduino_bridge arduino_bridge robot_sensors nv21_converter_pkg`
  - Or move `backup_packages/` outside the workspace so `colcon` doesn’t discover them.
- For simulation, ensure `robot_gazebo` is built when launching with `use_gazebo:=true`:
  - `colcon build --packages-select robot_gazebo`
  - `source install/setup.bash`

## 📊 System Monitoring

### Health Dashboards

The system provides comprehensive monitoring:

**Hardware Status:**
- Arduino connection state and data flow
- Camera frame rate and image quality
- LiDAR scan rate and data validity
- USB device connectivity

**Safety Status:**
- Emergency stop state
- Obstacle detection alerts
- Velocity limit violations
- Hardware error conditions

**Performance Metrics:**
- CPU and memory usage
- Network bandwidth utilization
- Message publication rates
- System response times

### Diagnostic Tools

```bash
# Real-time system status
ros2 run rqt_robot_monitor rqt_robot_monitor

# Topic monitoring
ros2 run rqt_topic rqt_topic

# Parameter management
ros2 run rqt_reconfigure rqt_reconfigure
```

## 🚀 Deployment

### Raspberry Pi Setup

1. **Install ROS2 Humble**
2. **Clone repository**
3. **Run build script**
4. **Configure autostart** (optional)

```bash
# Autostart on boot
sudo systemctl enable robot-startup.service
```

### Docker Deployment (Alternative)

```dockerfile
FROM ros:humble
COPY . /workspace
WORKDIR /workspace
RUN ./build_ros2_pi.sh
CMD ["ros2", "launch", "robot_bringup", "bringup.launch.py"]
```

## 📚 API Reference

### Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Robot velocity commands |
| `/odom` | `nav_msgs/Odometry` | Robot odometry |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan data |
| `/image_raw` | `sensor_msgs/Image` | Camera images |
| `/system_status` | `std_msgs/String` | Overall system health |
| `/emergency_stop` | `std_msgs/Bool` | Emergency stop state |
| `/servo_cmd` | `std_msgs/Bool` | Servo control(open 5s on true) |

### Key Services

| Service | Type | Description |
|---------|------|-------------|
| `/set_control_mode` | `robot_interfaces/SetMode` | Change control mode |
| `/calibrate_hardware` | `robot_interfaces/Calibration` | Hardware calibration |

### Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `max_linear_velocity` | 0.5 | Maximum forward speed (m/s) |
| `max_angular_velocity` | 1.0 | Maximum rotation speed (rad/s) |
| `safety_stop_distance` | 0.3 | Emergency stop distance (m) |
| `cmd_vel_timeout` | 1.0 | Command timeout (seconds) |
| `servo_open_time` | 5.0 | Auto-close delay | 

## 🤝 Contributing

1. **Fork the repository**
2. **Create feature branch**: `git checkout -b feature/amazing-feature`
3. **Follow coding standards**: Use ROS2 conventions
4. **Add tests**: Ensure new code is tested
5. **Update documentation**: Keep README current
6. **Submit pull request**: Describe changes clearly

### Coding Standards

- **Python**: Follow PEP 8, use type hints
- **ROS2**: Follow ROS2 style guide
- **Comments**: Document complex logic
- **Safety**: Always consider safety implications

## 📄 License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **ROS2 Community** - For the excellent robotics framework
- **Open Source Contributors** - For the libraries and tools used
- **Robotics Community** - For inspiration and best practices

---

**Need help?** Open an issue or check the troubleshooting section above.
