# Robot Car (ROS 2 Humble) Workspace

A comprehensive ROS 2 Humble workspace for a Raspberry Pi 4 robot car with Arduino integration, sensor processing, and autonomous navigation capabilities.

## Key Features

- **Arduino Integration**: Bidirectional communication between ROS 2 and Arduino for motor control and sensor reading
- **Sensor Suite**: Support for RPi Camera, LiDAR, and various analog/digital sensors
- **Computer Vision**: OpenCV-based image processing and object detection
- **Navigation**: Integration with Nav2 for autonomous navigation
- **Simulation**: Support for Ignition Gazebo Fortress
- **Modular Design**: Easily extensible architecture for adding new capabilities

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

### Optional Dependencies (for simulation)
```bash
sudo apt install -y \
  ros-humble-ros-ign-gazebo \
  ros-humble-ros-ign-bridge
```

## Workspace Structure

- `src/arduino_bridge/` - ROS 2 node for Arduino communication
  - `arduino_bridge_node.py` - Main node for serial communication
  - `launch/` - Launch configurations
  - `config/` - Configuration files

- `src/robot_bringup/` - Top-level launch files
- `src/robot_sensors/` - Camera and LiDAR nodes
- `src/robot_control/` - Motor control and other actuators
- `src/robot_perception/` - Computer vision processing
- `src/robot_description/` - URDF/Xacro models
- `src/robot_navigation/` - Navigation stack configuration
- `src/robot_simulation/` - Simulation environments

## Getting Started

### 1. Build the Workspace
```bash
source /opt/ros/humble/setup.bash
cd ~/Dojo
colcon build --symlink-install
source install/setup.bash
```

### 2. Arduino Setup
1. Upload the provided Arduino sketch to your board
2. Connect the Arduino to the Raspberry Pi via USB
3. Verify the connection:
   ```bash
   ls /dev/ttyACM*
   ```

### 3. Run the Arduino Bridge
```bash
# Terminal 1: Start the bridge node
ros2 run arduino_bridge arduino_bridge_node

# Terminal 2: Control the LED
ros2 topic pub /led_control std_msgs/msg/Int32 "data: 1" -1  # Turn on
ros2 topic pub /led_control std_msgs/msg/Int32 "data: 0" -1  # Turn off

# Terminal 3: View sensor data
ros2 topic echo /sensor_data
```

### 4. Run the Complete System
```bash
# Start all nodes
ros2 launch robot_bringup bringup.launch.py
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
1. **Permission denied** on serial port:
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and log back in
   ```

2. **Arduino not detected**:
   - Check USB connection
   - Verify the correct port in the launch file
   - Try resetting the Arduino

3. **ROS 2 nodes not found**:
   ```bash
   source install/setup.bash
   ```

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please read our [contributing guidelines](CONTRIBUTING.md) before submitting pull requests.
