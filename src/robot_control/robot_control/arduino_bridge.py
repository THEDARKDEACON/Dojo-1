#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
import serial
import serial.tools.list_ports
import time
import yaml
import os

from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Imu, Range
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 57600)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('write_timeout', 1.0)
        self.declare_parameter('start_delimiter', '<')
        self.declare_parameter('end_delimiter', '>')
        self.declare_parameter('field_separator', ',')
        self.declare_parameter('debug', True)
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.write_timeout = self.get_parameter('write_timeout').value
        self.start_delimiter = self.get_parameter('start_delimiter').value
        self.end_delimiter = self.get_parameter('end_delimiter').value
        self.field_separator = self.get_parameter('field_separator').value
        self.debug = self.get_parameter('debug').value
        
        # Serial connection
        self.serial_conn = None
        self.connected = False
        
        # Motor commands
        self.left_motor_cmd = 0
        self.right_motor_cmd = 0
        
        # Encoder data
        self.left_encoder = 0
        self.right_encoder = 0
        self.last_encoder_left = 0
        self.last_encoder_right = 0
        self.last_encoder_time = self.get_clock().now()
        
        # Odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_update = self.get_clock().now()
        
        # Setup publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.left_encoder_pub = self.create_publisher(Int32, 'encoder/left', 10)
        self.right_encoder_pub = self.create_publisher(Int32, 'encoder/right', 10)
        
        # Setup subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for reading from serial
        self.serial_timer = self.create_timer(0.01, self.serial_read_callback)  # 100Hz
        
        # Connect to Arduino
        self.connect_arduino()
        
        self.get_logger().info('Arduino bridge node started')
    
    def connect_arduino(self):
        """Attempt to connect to the Arduino."""
        self.get_logger().info(f'Connecting to Arduino on {self.port}...')
        
        if not os.path.exists(self.port):
            self.get_logger().warn(f'Port {self.port} does not exist. Searching for Arduino...')
            self.find_arduino()
        
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
                write_timeout=self.write_timeout
            )
            # Wait for Arduino to reset
            time.sleep(2)
            self.connected = True
            self.get_logger().info(f'Connected to Arduino on {self.port}')
            
            # Send initial configuration
            self.send_config()
            
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino: {str(e)}')
            self.connected = False
    
    def find_arduino(self):
        """Try to find Arduino by checking common ports and vendor IDs."""
        # Common Arduino vendor IDs
        arduino_vids = {
            0x2341,  # Arduino
            0x2a03,  # Arduino.org
            0x1a86,  # CH340 (common clone)
            0x10C4,  # CP210x
            0x0403,  # FTDI
        }
        
        for port in serial.tools.list_ports.comports():
            if port.vid in arduino_vids:
                self.port = port.device
                self.get_logger().info(f'Found Arduino on {self.port}')
                return
        
        self.get_logger().error('Could not find Arduino. Please check connections.')
    
    def send_config(self):
        """Send configuration to Arduino."""
        if not self.connected:
            return
            
        # Example: Send PID parameters
        # Format: <CONFIG,PID,Kp,Ki,Kd>
        config_msg = f"{self.start_delimiter}CONFIG,PID,1.0,0.1,0.05{self.end_delimiter}\n"
        self.serial_write(config_msg)
    
    def cmd_vel_callback(self, msg):
        """Handle incoming cmd_vel messages."""
        # Convert twist to motor commands (differential drive)
        # This is a simple implementation - you might want to use the cmd_vel_to_motors node instead
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Simple differential drive model
        wheel_sep = 0.3  # meters
        wheel_radius = 0.05  # meters
        
        # Convert to wheel speeds (rad/s)
        left_speed = (linear - angular * wheel_sep / 2.0) / wheel_radius
        right_speed = (linear + angular * wheel_sep / 2.0) / wheel_radius
        
        # Convert to PWM values (simplified)
        max_pwm = 255
        max_speed = 6.28  # rad/s (adjust based on your motor)
        
        self.left_motor_cmd = int((left_speed / max_speed) * max_pwm)
        self.right_motor_cmd = int((right_speed / max_speed) * max_pwm)
        
        # Clamp values
        self.left_motor_cmd = max(-max_pwm, min(max_pwm, self.left_motor_cmd))
        self.right_motor_cmd = max(-max_pwm, min(max_pwm, self.right_motor_cmd))
        
        # Send motor commands to Arduino
        self.send_motor_commands()
    
    def send_motor_commands(self):
        """Send motor commands to Arduino."""
        if not self.connected:
            return
            
        # Format: <LEFT_MOTOR,RIGHT_MOTOR> (e.g., <100,-100>)
        motor_cmd = f"{self.start_delimiter}{self.left_motor_cmd}{self.field_separator}{self.right_motor_cmd}{self.end_delimiter}\n"
        self.serial_write(motor_cmd)
    
    def serial_write(self, data):
        """Safely write data to the serial port."""
        try:
            if self.connected and self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.write(data.encode('utf-8'))
                return True
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {str(e)}')
            self.connected = False
        return False
    
    def serial_read_callback(self):
        """Read and process data from the serial port."""
        if not self.connected:
            # Try to reconnect
            self.connect_arduino()
            return
            
        try:
            if self.serial_conn.in_waiting > 0:
                # Read a line from the serial port
                line = self.serial_conn.readline().decode('utf-8').strip()
                
                # Process the line if it contains our delimiters
                if line.startswith(self.start_delimiter) and line.endswith(self.end_delimiter):
                    # Remove delimiters and split by separator
                    content = line[1:-1]  # Remove < and >
                    parts = content.split(self.field_separator)
                    
                    # Process different message types
                    if len(parts) >= 2 and parts[0] == 'ENCODER':
                        self.process_encoder_data(parts[1:])
                    elif len(parts) >= 3 and parts[0] == 'IMU':
                        self.process_imu_data(parts[1:])
                    elif len(parts) >= 1 and parts[0] == 'PING':
                        self.get_logger().debug('Received PING from Arduino')
                    
                    # Debug output
                    if self.debug:
                        self.get_logger().debug(f'Received: {line}')
                        
        except UnicodeDecodeError:
            self.get_logger().warn('Received invalid data from Arduino')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read error: {str(e)}')
            self.connected = False
    
    def process_encoder_data(self, data):
        """Process encoder data from Arduino."""
        if len(data) < 2:
            return
            
        try:
            # Parse encoder values
            left_ticks = int(data[0])
            right_ticks = int(data[1])
            
            # Update encoder values
            self.left_encoder = left_ticks
            self.right_encoder = right_ticks
            
            # Publish encoder values
            left_msg = Int32()
            left_msg.data = left_ticks
            self.left_encoder_pub.publish(left_msg)
            
            right_msg = Int32()
            right_msg.data = right_ticks
            self.right_encoder_pub.publish(right_msg)
            
            # Update odometry
            self.update_odometry(left_ticks, right_ticks)
            
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Failed to parse encoder data: {data}, error: {str(e)}')
    
    def update_odometry(self, left_ticks, right_ticks):
        """Update robot odometry based on encoder ticks."""
        # Get current time
        current_time = self.get_clock().now()
        dt = (current_time - self.last_odom_update).nanoseconds / 1e9  # Convert to seconds
        
        if dt <= 0:
            return
            
        # Calculate distance traveled by each wheel
        # ticks_per_rev = 20  # Update this based on your encoder
        # wheel_circumference = 0.314  # meters (2 * pi * radius)
        # distance_per_tick = wheel_circumference / ticks_per_rev
        
        # For now, just use the difference in ticks
        left_delta = left_ticks - self.last_encoder_left
        right_delta = right_ticks - self.last_encoder_right
        
        # Update last encoder values
        self.last_encoder_left = left_ticks
        self.last_encoder_right = right_ticks
        self.last_odom_update = current_time
        
        # Simple odometry calculation (improve with proper model)
        # This is a placeholder - you should implement proper odometry calculation
        # based on your robot's kinematics and encoder resolution
        
        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        # Set position (placeholder values)
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Set orientation (quaternion from yaw)
        from math import sin, cos
        from geometry_msgs.msg import Quaternion
        odom_msg.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=sin(self.theta / 2.0),
            w=cos(self.theta / 2.0)
        )
        
        # Set twist (placeholder values)
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        
        # Publish the odometry message
        self.odom_pub.publish(odom_msg)
        
        # Publish transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_msg.pose.pose.orientation
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)
    
    def process_imu_data(self, data):
        """Process IMU data from Arduino (if available)."""
        # Implement IMU data processing if your Arduino has an IMU
        pass
    
    def __del__(self):
        """Cleanup on node destruction."""
        if hasattr(self, 'serial_conn') and self.serial_conn and self.serial_conn.is_open:
            try:
                # Stop motors before exiting
                self.left_motor_cmd = 0
                self.right_motor_cmd = 0
                self.send_motor_commands()
                self.serial_conn.close()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ArduinoBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
