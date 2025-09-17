#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32
import serial
import time
from threading import Lock

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 1.0)
        
        # Get parameters
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        
        # Initialize serial connection
        self.serial_lock = Lock()
        try:
            self.serial = serial.Serial(port, baudrate, timeout=timeout)
            self.get_logger().info(f'Connected to Arduino on {port} at {baudrate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Could not connect to Arduino: {e}')
            raise
        
        # Wait for Arduino to reset
        time.sleep(2.0)
        
        # Publishers
        self.left_encoder_pub = self.create_publisher(Int32, 'left_encoder', 10)
        self.right_encoder_pub = self.create_publisher(Int32, 'right_encoder', 10)
        self.left_speed_pub = self.create_publisher(Float32, 'left_wheel_speed', 10)
        self.right_speed_pub = self.create_publisher(Float32, 'right_wheel_speed', 10)
        
        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Timer for reading serial data
        self.timer = self.create_timer(0.01, self.read_serial)  # 100Hz
        
        # Robot parameters (adjust these based on your robot)
        self.wheel_radius = 0.1  # meters
        self.wheel_separation = 0.5  # meters
        
        # Send initial zero command
        self.send_motor_command(0.0, 0.0)
        
        self.get_logger().info('Arduino bridge node started')
    
    def cmd_vel_callback(self, msg):
        try:
            # Convert twist to wheel speeds (rad/s)
            linear = msg.linear.x
            angular = msg.angular.z
            
            # Calculate wheel speeds using differential drive kinematics
            left_speed = (linear - angular * self.wheel_separation / 2) / self.wheel_radius
            right_speed = (linear + angular * self.wheel_separation / 2) / self.wheel_radius
            
            # Send motor commands to Arduino
            self.send_motor_command(left_speed, right_speed)
        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel_callback: {str(e)}')
    
    def send_motor_command(self, left_speed, right_speed):
        """
        Send motor speed commands to the Arduino.
        
        Args:
            left_speed (float): Left wheel speed in rad/s
            right_speed (float): Right wheel speed in rad/s
        """
        try:
            # Limit speeds to reasonable values (adjust as needed)
            left_speed = max(min(left_speed, 10.0), -10.0)  # ±10 rad/s limit
            right_speed = max(min(right_speed, 10.0), -10.0)
            
            # Format the command
            cmd = f"M{left_speed:.4f},{right_speed:.4f}\n".encode('ascii')
            
            # Send the command
            with self.serial_lock:
                try:
                    self.serial.write(cmd)
                    self.serial.flush()
                except serial.SerialException as e:
                    self.get_logger().error(f'Error sending command to Arduino: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in send_motor_command: {str(e)}')
    
    def read_serial(self):
        """
        Read data from Arduino and publish to ROS2 topics
        
        Expected message formats:
        - E<left_ticks>,<right_ticks>,<left_speed>,<right_speed>
        - D<debug_message>
        - V<voltage>
        """
        with self.serial_lock:
            if not self.serial.in_waiting:
                return
                
            try:
                line = self.serial.readline().decode('ascii', errors='ignore').strip()
                if not line:
                    return
                    
                # Debug output from Arduino (starts with 'D')
                if line.startswith('D'):
                    self.get_logger().debug(f"Arduino: {line[1:]}")
                    return
                
                # Handle encoder data
                if line.startswith('E'):
                    try:
                        # Format: "E<left_ticks>,<right_ticks>,<left_speed>,<right_speed>"
                        parts = line[1:].split(',')
                        if len(parts) == 4:
                            left_ticks = int(parts[0])
                            right_ticks = int(parts[1])
                            left_speed = float(parts[2])
                            right_speed = float(parts[3])
                            
                            # Publish encoder ticks
                            left_msg = Int32()
                            left_msg.data = left_ticks
                            self.left_encoder_pub.publish(left_msg)
                            
                            right_msg = Int32()
                            right_msg.data = right_ticks
                            self.right_encoder_pub.publish(right_msg)
                            
                            # Publish wheel speeds
                            left_speed_msg = Float32()
                            left_speed_msg.data = left_speed
                            self.left_speed_pub.publish(left_speed_msg)
                            
                            right_speed_msg = Float32()
                            right_speed_msg.data = right_speed
                            self.right_speed_pub.publish(right_speed_msg)
                            
                            # Debug output (throttled to avoid flooding)
                            self.get_logger().debug(
                                f'Encoders - L: {left_ticks} ticks ({left_speed:.2f} rad/s), ' + 
                                f'R: {right_ticks} ticks ({right_speed:.2f} rad/s)',
                                throttle_duration_sec=0.5
                            )
                    except (ValueError, IndexError) as e:
                        self.get_logger().warn(f'Invalid encoder data: {line}')
                
                # Handle battery voltage (if implemented on Arduino)
                elif line.startswith('V'):
                    try:
                        voltage = float(line[1:])
                        if hasattr(self, 'voltage_pub'):
                            voltage_msg = Float32()
                            voltage_msg.data = voltage
                            self.voltage_pub.publish(voltage_msg)
                        self.get_logger().debug(f'Battery voltage: {voltage:.2f}V')
                    except ValueError as e:
                        self.get_logger().warn(f'Invalid voltage data: {line}')
                        
            except UnicodeDecodeError as e:
                self.get_logger().warn(f'Failed to decode serial data: {e}')
            except Exception as e:
                self.get_logger().error(f'Error in read_serial: {str(e)}')

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
