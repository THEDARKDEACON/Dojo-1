#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import serial
import time

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        
        # Setup serial connection
        try:
            self.serial_connection = serial.Serial(self.port, self.baud_rate, timeout=1)
            time.sleep(2)  # Wait for Arduino to reset
            self.get_logger().info(f'Connected to Arduino on {self.port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {str(e)}')
            raise
        
        # Publisher for sensor data
        self.sensor_pub = self.create_publisher(Float32, 'sensor_data', 10)
        
        # Subscriber for LED control
        self.led_sub = self.create_subscription(
            Int32,
            'led_control',
            self.led_callback,
            10
        )
        
        # Timer for reading from Arduino
        self.timer = self.create_timer(0.1, self.read_serial)
    
    def led_callback(self, msg):
        """Send LED command to Arduino"""
        try:
            self.serial_connection.write(f"L{msg.data}\n".encode())
        except Exception as e:
            self.get_logger().error(f'Error sending to Arduino: {str(e)}')
    
    def read_serial(self):
        """Read data from Arduino and publish to ROS2 topics"""
        if self.serial_connection.in_waiting:
            try:
                line = self.serial_connection.readline().decode('utf-8').strip()
                if line.startswith('S'):  # Sensor data
                    try:
                        sensor_value = float(line[1:])
                        msg = Float32()
                        msg.data = sensor_value
                        self.sensor_pub.publish(msg)
                    except ValueError:
                        self.get_logger().warn(f'Invalid sensor data: {line}')
            except Exception as e:
                self.get_logger().error(f'Error reading from Arduino: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
