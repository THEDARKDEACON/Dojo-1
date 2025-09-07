#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String
from sensor_msgs.msg import Imu, Range, JointState
import serial
import serial.tools.list_ports
import time
import threading
import json

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Parameters
        self.declare_parameter('port', 'auto')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('sensor_publish_rate', 10.0)
        
        self.port = self.get_parameter('port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.sensor_publish_rate = self.get_parameter('sensor_publish_rate').value
        
        # Setup serial connection
        self.serial_connection = None
        self.connect_to_arduino()
        
        # Publishers
        self.encoder_pub = self.create_publisher(JointState, 'wheel_encoders', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.range_pub = self.create_publisher(Range, 'ultrasonic', 10)
        self.debug_pub = self.create_publisher(String, 'arduino_debug', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            JointState,
            'wheel_commands',
            self.cmd_vel_callback,
            10)
            
        # Timer for sensor reading
        self.sensor_timer = self.create_timer(
            1.0 / self.sensor_publish_rate,
            self.read_sensors)
            
        # Thread for reading serial data
        self.serial_thread = threading.Thread(target=self.read_serial_thread)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        self.get_logger().info('Arduino Bridge Node started')
    
    def connect_to_arduino(self):
        """Connect to Arduino board"""
        if self.port == 'auto':
            self.port = self.find_arduino_port()
            if not self.port:
                self.get_logger().error('Could not find Arduino port')
                return False
        
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.timeout)
            
            # Wait for Arduino to reset
            time.sleep(2)
            self.get_logger().info(f'Connected to Arduino on {self.port}')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {str(e)}')
            self.serial_connection = None
            return False
    
    def find_arduino_port(self):
        """Find Arduino port automatically"""
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if 'Arduino' in p.description or 'ttyACM' in p.device:
                self.get_logger().info(f'Found Arduino at {p.device}')
                return p.device
        return None
    
    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands"""
        if not self.serial_connection or not self.serial_connection.is_open:
            self.get_logger().warn('Serial port not open, cannot send command')
            return
            
        try:
            # Format: CMD:LEFT_SPEED,RIGHT_SPEED\n
            # Create a dictionary with the command
            cmd = {
                'type': 'cmd_vel',
                'left_speed': msg.velocity[0] if len(msg.velocity) > 0 else 0.0,
                'right_speed': msg.velocity[1] if len(msg.velocity) > 1 else 0.0
            }
            
            # Convert to JSON and send
            cmd_str = json.dumps(cmd) + '\n'
            self.serial_connection.write(cmd_str.encode('utf-8'))
            
        except Exception as e:
            self.get_logger().error(f'Error sending command: {str(e)}')
    
    def read_serial_thread(self):
        """Thread for reading serial data"""
        buffer = ""
        while rclpy.ok():
            if not self.serial_connection or not self.serial_connection.is_open:
                time.sleep(1)
                continue
                
            try:
                # Read all available data
                while self.serial_connection.in_waiting > 0:
                    char = self.serial_connection.read().decode('utf-8', errors='ignore')
                    if char == '\n':
                        self.process_serial_data(buffer)
                        buffer = ""
                    else:
                        buffer += char
                
            except Exception as e:
                self.get_logger().error(f'Serial read error: {str(e)}')
                time.sleep(1)
                self.connect_to_arduino() if not self.serial_connection else None
    
    def process_serial_data(self, data):
        """Process incoming serial data"""
        try:
            # Parse JSON data
            msg = json.loads(data)
            
            if msg.get('type') == 'sensor_data':
                # Publish sensor data
                self.publish_sensor_data(msg)
                
            elif msg.get('type') == 'debug':
                # Publish debug message
                debug_msg = String()
                debug_msg.data = msg.get('message', '')
                self.debug_pub.publish(debug_msg)
                
        except json.JSONDecodeError:
            self.get_logger().warn(f'Invalid JSON: {data}')
        except Exception as e:
            self.get_logger().error(f'Error processing serial data: {str(e)}')
    
    def publish_sensor_data(self, data):
        """Publish sensor data to ROS topics"""
        # Publish encoder data
        if 'encoders' in data:
            encoder_msg = JointState()
            encoder_msg.header.stamp = self.get_clock().now().to_msg()
            encoder_msg.name = ['left_wheel_joint', 'right_wheel_joint']
            encoder_msg.position = [
                data['encoders'].get('left', 0.0),
                data['encoders'].get('right', 0.0)
            ]
            self.encoder_pub.publish(encoder_msg)
        
        # Publish IMU data
        if 'imu' in data:
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            
            imu = data['imu']
            if 'orientation' in imu:
                imu_msg.orientation.x = imu['orientation'].get('x', 0.0)
                imu_msg.orientation.y = imu['orientation'].get('y', 0.0)
                imu_msg.orientation.z = imu['orientation'].get('z', 0.0)
                imu_msg.orientation.w = imu['orientation'].get('w', 1.0)
                
            if 'angular_velocity' in imu:
                imu_msg.angular_velocity.x = imu['angular_velocity'].get('x', 0.0)
                imu_msg.angular_velocity.y = imu['angular_velocity'].get('y', 0.0)
                imu_msg.angular_velocity.z = imu['angular_velocity'].get('z', 0.0)
                
            if 'linear_acceleration' in imu:
                imu_msg.linear_acceleration.x = imu['linear_acceleration'].get('x', 0.0)
                imu_msg.linear_acceleration.y = imu['linear_acceleration'].get('y', 0.0)
                imu_msg.linear_acceleration.z = imu['linear_acceleration'].get('z', 0.0)
                
            self.imu_pub.publish(imu_msg)
        
        # Publish ultrasonic sensor data
        if 'ultrasonic' in data:
            range_msg = Range()
            range_msg.header.stamp = self.get_clock().now().to_msg()
            range_msg.header.frame_id = 'ultrasonic_sensor'
            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.field_of_view = 0.1  # 0.1 radians (~5.7 degrees)
            range_msg.min_range = 0.02  # 2cm
            range_msg.max_range = 4.0    # 4m
            range_msg.range = data['ultrasonic'].get('distance', 0.0)
            self.range_pub.publish(range_msg)
    
    def read_sensors(self):
        """Request sensor data from Arduino"""
        if not self.serial_connection or not self.serial_connection.is_open:
            return
            
        try:
            # Send a request for sensor data
            cmd = {'type': 'sensor_request'}
            self.serial_connection.write((json.dumps(cmd) + '\n').encode('utf-8'))
            
        except Exception as e:
            self.get_logger().error(f'Error requesting sensor data: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_connection and node.serial_connection.is_open:
            node.serial_connection.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
