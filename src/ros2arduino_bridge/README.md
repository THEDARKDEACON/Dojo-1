# ROS 2 Arduino Bridge

This package provides a bridge between ROS 2 and Arduino for motor control and sensor reading. It enables bidirectional communication between a ROS 2 network and an Arduino board, allowing for precise motor control and sensor data collection.

## Features

- **Bidirectional Communication**: JSON-based protocol over serial
- **Motor Control**: PID-based speed control for DC motors
- **Encoder Feedback**: Tracks wheel position and velocity
- **Modular Design**: Easy to extend with additional sensors
- **Testing Framework**: Comprehensive test suite for validation
- **Simulation Support**: Works with Gazebo simulation

## Hardware Requirements

- **Arduino Board**: Mega 2560 recommended
- **Motor Driver**: L298N or similar
- **Encoders**: Quadrature encoders for wheel feedback
- **USB Connection**: For serial communication

## Installation

### 1. Arduino Setup

1. Install required libraries:
   - [ArduinoJson](https://arduinojson.org/)
   - [PID_v1](https://playground.arduino.cc/Code/PIDLibrary/)

2. Upload the Arduino sketch:
   ```bash
   # Navigate to the Arduino sketch directory
   cd /path/to/Dojo/src/ros2arduino_bridge/arduino/ros2arduino_bridge/
   # Upload using arduino-cli or Arduino IDE
   arduino-cli compile --fqbn arduino:avr:mega ros2arduino_bridge.ino
   arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega ros2arduino_bridge.ino
   ```

### 2. ROS 2 Setup

1. Clone and build the package:
   ```bash
   # Navigate to your ROS 2 workspace
   cd ~/ros2_ws/src
   
   # Clone the repository
   git clone <repository-url>
   
   # Install dependencies
   sudo apt update
   sudo apt install python3-pip
   pip3 install pyserial numpy
   
   # Build the package
   cd ~/ros2_ws
   colcon build --packages-select ros2arduino_bridge
   source install/setup.bash
   ```

## Usage

### Starting the Bridge

```bash
# Basic usage
ros2 launch ros2arduino_bridge arduino_bridge.launch.py

# With custom port and baud rate
ros2 launch ros2arduino_bridge arduino_bridge.launch.py port:=/dev/ttyACM0 baud_rate:=115200
```

### Testing the Bridge

Run the test suite:

```bash
# Run all tests
./test_arduino_bridge.sh

# With custom parameters
./test_arduino_bridge.sh -p /dev/ttyACM0 -b 115200 -d 20
```

### Manual Testing

1. Check node status:
   ```bash
   ros2 node list
   ros2 topic list
   ```

2. Send test commands:
   ```bash
   # Send a command to move forward
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear:
     x: 0.2
     y: 0.0
     z: 0.0
   angular:
     x: 0.0
     y: 0.0
     z: 0.0"
   ```

3. View encoder data:
   ```bash
   ros2 topic echo /wheel/encoders
   ```

## Testing Framework

The package includes a comprehensive testing framework:

1. **Unit Tests**: Test individual components
2. **Integration Tests**: Test the full system
3. **Hardware Tests**: Validate hardware communication

Run tests with:
```bash
colcon test --packages-select ros2arduino_bridge
```

## Simulation

To use with Gazebo simulation:

```bash
ros2 launch ros2arduino_bridge arduino_bridge.launch.py use_sim_time:=true
```

## Troubleshooting

### Common Issues

1. **Permission Denied** on serial port:
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and log back in
   ```

2. **Node not found**:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

3. **Serial Communication Errors**:
   - Check baud rate matches on both ends
   - Verify cable connection
   - Check for other processes using the port

## License

Apache 2.0
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
