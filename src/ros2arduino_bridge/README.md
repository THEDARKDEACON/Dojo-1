# ROS 2 Arduino Bridge

This package provides a bridge between ROS 2 and Arduino using the ros2arduino library. It enables bidirectional communication between a ROS 2 network and an Arduino board, allowing for sensor data collection and motor control.

## Features

- Bidirectional communication between ROS 2 and Arduino
- Support for multiple sensor types (IMU, encoders, ultrasonic, etc.)
- Motor control interface
- Configurable parameters for different hardware setups
- Launch file for easy startup

## Hardware Requirements

- Arduino board (Uno, Mega, or similar)
- Motor driver board (compatible with Arduino)
- Sensors (IMU, encoders, ultrasonic, etc.)
- USB connection to ROS 2 machine

## Installation

### Arduino Setup

1. Install the ros2arduino library in your Arduino IDE:
   - Download the latest release from [ros2arduino GitHub](https://github.com/ROBOTIS-GIT/ros2arduino)
   - Install the library in your Arduino IDE (Sketch > Include Library > Add .ZIP Library)

2. Upload the Arduino sketch:
   - Open `arduino/ros2arduino_bridge/ros2arduino_bridge.ino` in Arduino IDE
   - Select your board and port
   - Upload the sketch

### ROS 2 Setup

1. Clone this package to your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone <repository-url> ros2arduino_bridge
   ```

2. Install dependencies:
   ```bash
   sudo apt update
   sudo apt install python3-pip
   pip3 install pyserial
   ```

3. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ros2arduino_bridge
   source install/setup.bash
   ```

## Usage

### Starting the Bridge

```bash
# Source your ROS 2 workspace
source ~/ros2_ws/install/setup.bash

# Start the bridge node
ros2 launch ros2arduino_bridge arduino_bridge.launch.py
```

### Topics

#### Subscribed Topics
- `/cmd_vel` (`geometry_msgs/msg/Twist`): Velocity commands for the robot

#### Published Topics
- `/wheel_encoders` (`sensor_msgs/msg/JointState`): Encoder readings
- `/imu/data` (`sensor_msgs/msg/Imu`): IMU data
- `/ultrasonic` (`sensor_msgs/msg/Range`): Ultrasonic sensor readings
- `/arduino_debug` (`std_msgs/msg/String`): Debug messages from the Arduino

### Parameters

- `port` (string, default: '/dev/ttyACM0'): Serial port for Arduino
- `baud_rate` (int, default: 115200): Baud rate for serial communication
- `timeout` (float, default: 1.0): Serial timeout in seconds
- `sensor_publish_rate` (float, default: 10.0): Rate for publishing sensor data (Hz)

## Customization

### Adding New Sensors

1. Update the Arduino sketch to read from your sensor
2. Add a new publisher in the Arduino code
3. Update the ROS 2 node to handle the new sensor data
4. Add new message types if needed

### Changing Motor Control

Modify the `controlMotors()` function in the Arduino sketch to match your motor driver and wiring.

## Troubleshooting

### Common Issues

1. **Permission denied on serial port**:
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
   source ~/ros2_ws/install/setup.bash
   ```

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.
