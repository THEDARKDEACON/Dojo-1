#!/usr/bin/env python3
"""
Unified Arduino Driver for Dojo Robot
Consolidates functionality from arduino_bridge, ros2arduino_bridge, and robot_control
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
import serial
import serial.tools.list_ports
import time
import threading
import json

from std_msgs.msg import Int32, Float32, String
from sensor_msgs.msg import Imu, Range
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class ArduinoDriver(Node):
    """Unified Arduino communication driver"""
    
    def __init__(self):
        super().__init__('arduino_driver')
        
        # Declare parameters with defaults
        self._declare_parameters()
        
        # Initialize hardware connection
        self.serial_conn = None
        self.connected = False
        self.connection_lock = threading.Lock()
        
        # Initialize data storage
        self._init_data_storage()
        
        # Setup ROS interfaces
        self._setup_publishers()
        self._setup_subscribers()
        
        # Start connection and communication threads
        self._start_communication()
        
        self.get_logger().info('Arduino Driver initialized')
    
    def _declare_parameters(self):
        """Declare all ROS parameters"""
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('write_timeout', 1.0)
        self.declare_parameter('start_delimiter', '<')
        self.declare_parameter('end_delimiter', '>')
        self.declare_parameter('field_separator', ',')
        self.declare_parameter('debug', True)
        self.declare_parameter('reconnect_interval', 5.0)
        
        # Motor parameters
        self.declare_parameter('motor_max', 255)
        self.declare_parameter('motor_min', 0)
        self.declare_parameter('wheel_base', 0.2)
        self.declare_parameter('wheel_radius', 0.033)
        
        # Encoder parameters  
        self.declare_parameter('encoder_ticks_per_rev', 20)
        self.declare_parameter('wheel_circumference', 0.314) 
   
    def _init_data_storage(self):
        """Initialize data storage variables"""
        self.motor_commands = {'left': 0, 'right': 0}
        self.encoder_data = {'left': 0, 'right': 0}
        self.sensor_data = {}
        self.last_encoder_time = time.time()
        
    def _setup_publishers(self):
        """Setup ROS publishers"""
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.range_pub = self.create_publisher(Range, 'ultrasonic', 10)
        self.status_pub = self.create_publisher(String, 'arduino_status', 10)
        
        # TF broadcaster for odometry
        self.tf_broadcaster = TransformBroadcaster(self)
        
    def _setup_subscribers(self):
        """Setup ROS subscribers"""
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self._cmd_vel_callback, 10)
        
    def _start_communication(self):
        """Start communication threads"""
        # Connection management thread
        self.connection_thread = threading.Thread(target=self._manage_connection)
        self.connection_thread.daemon = True
        self.connection_thread.start()
        
        # Data reading thread
        self.read_thread = threading.Thread(target=self._read_data_loop)
        self.read_thread.daemon = True
        self.read_thread.start()
        
    def _manage_connection(self):
        """Manage Arduino connection with auto-reconnect"""
        while rclpy.ok():
            if not self.connected:
                self._attempt_connection()
            time.sleep(self.get_parameter('reconnect_interval').value)
    
    def _attempt_connection(self):
        """Attempt to connect to Arduino"""
        try:
            port = self.get_parameter('port').value
            baud_rate = self.get_parameter('baud_rate').value
            timeout = self.get_parameter('timeout').value
            
            with self.connection_lock:
                if self.serial_conn:
                    self.serial_conn.close()
                
                self.serial_conn = serial.Serial(
                    port=port,
                    baudrate=baud_rate,
                    timeout=timeout,
                    write_timeout=self.get_parameter('write_timeout').value
                )
                
                self.connected = True
                self.get_logger().info(f'Connected to Arduino on {port}')
                self._publish_status('CONNECTED')
                
        except Exception as e:
            self.connected = False
            self.get_logger().warn(f'Failed to connect to Arduino: {e}')
            self._publish_status('DISCONNECTED')  
  
    def _read_data_loop(self):
        """Main data reading loop"""
        while rclpy.ok():
            if self.connected and self.serial_conn:
                try:
                    self._read_arduino_data()
                except Exception as e:
                    self.get_logger().error(f'Error reading Arduino data: {e}')
                    self.connected = False
            time.sleep(0.01)  # 100Hz reading rate
    
    def _read_arduino_data(self):
        """Read and parse data from Arduino"""
        if not self.serial_conn or not self.serial_conn.in_waiting:
            return
            
        try:
            line = self.serial_conn.readline().decode('utf-8').strip()
            if line.startswith(self.get_parameter('start_delimiter').value) and \
               line.endswith(self.get_parameter('end_delimiter').value):
                
                # Remove delimiters and parse
                data_str = line[1:-1]  # Remove < and >
                self._parse_sensor_data(data_str)
                
        except Exception as e:
            self.get_logger().debug(f'Parse error: {e}')
    
    def _parse_sensor_data(self, data_str):
        """Parse sensor data from Arduino"""
        try:
            separator = self.get_parameter('field_separator').value
            fields = data_str.split(separator)
            
            if len(fields) >= 4:
                # Expected format: <encoder_left,encoder_right,ultrasonic,imu_data>
                encoder_left = int(fields[0])
                encoder_right = int(fields[1])
                ultrasonic = float(fields[2])
                
                # Update encoder data
                self._update_encoders(encoder_left, encoder_right)
                
                # Publish ultrasonic data
                self._publish_ultrasonic(ultrasonic)
                
                # Parse additional IMU data if available
                if len(fields) > 3:
                    self._parse_imu_data(fields[3:])
                    
        except (ValueError, IndexError) as e:
            self.get_logger().debug(f'Data parsing error: {e}')
    
    def _update_encoders(self, left_ticks, right_ticks):
        """Update encoder data and publish odometry"""
        current_time = time.time()
        dt = current_time - self.last_encoder_time
        
        if dt > 0:
            # Calculate wheel velocities
            ticks_per_rev = self.get_parameter('encoder_ticks_per_rev').value
            wheel_circumference = self.get_parameter('wheel_circumference').value
            
            left_delta = left_ticks - self.encoder_data['left']
            right_delta = right_ticks - self.encoder_data['right']
            
            left_velocity = (left_delta / ticks_per_rev) * wheel_circumference / dt
            right_velocity = (right_delta / ticks_per_rev) * wheel_circumference / dt
            
            # Update stored values
            self.encoder_data['left'] = left_ticks
            self.encoder_data['right'] = right_ticks
            self.last_encoder_time = current_time
            
            # Publish odometry
            self._publish_odometry(left_velocity, right_velocity, current_time) 
   
    def _publish_odometry(self, left_vel, right_vel, timestamp):
        """Publish odometry data"""
        # Calculate robot velocities
        wheel_base = self.get_parameter('wheel_base').value
        linear_vel = (left_vel + right_vel) / 2.0
        angular_vel = (right_vel - left_vel) / wheel_base
        
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Set velocities
        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.angular.z = angular_vel
        
        self.odom_pub.publish(odom_msg)
    
    def _publish_ultrasonic(self, distance):
        """Publish ultrasonic sensor data"""
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.header.frame_id = 'ultrasonic_link'
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.26  # ~15 degrees
        range_msg.min_range = 0.02
        range_msg.max_range = 4.0
        range_msg.range = distance / 100.0  # Convert cm to meters
        
        self.range_pub.publish(range_msg)
    
    def _parse_imu_data(self, imu_fields):
        """Parse and publish IMU data if available"""
        # Placeholder for IMU data parsing
        # Implement based on your Arduino's IMU output format
        pass
    
    def _publish_status(self, status):
        """Publish Arduino connection status"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)
    
    def _cmd_vel_callback(self, msg):
        """Handle cmd_vel messages and send to Arduino"""
        if not self.connected:
            return
            
        # Convert twist to motor commands
        linear = msg.linear.x
        angular = msg.angular.z
        wheel_base = self.get_parameter('wheel_base').value
        wheel_radius = self.get_parameter('wheel_radius').value
        
        # Calculate wheel velocities
        left_vel = linear - (angular * wheel_base / 2.0)
        right_vel = linear + (angular * wheel_base / 2.0)
        
        # Convert to motor PWM values
        motor_max = self.get_parameter('motor_max').value
        left_pwm = int(max(-motor_max, min(motor_max, left_vel * 100)))
        right_pwm = int(max(-motor_max, min(motor_max, right_vel * 100)))
        
        # Send to Arduino
        self._send_motor_commands(left_pwm, right_pwm)    

    def _send_motor_commands(self, left_pwm, right_pwm):
        """Send motor commands to Arduino"""
        if not self.connected or not self.serial_conn:
            return
            
        try:
            start_delim = self.get_parameter('start_delimiter').value
            end_delim = self.get_parameter('end_delimiter').value
            separator = self.get_parameter('field_separator').value
            
            command = f"{start_delim}{left_pwm}{separator}{right_pwm}{end_delim}\n"
            
            with self.connection_lock:
                self.serial_conn.write(command.encode('utf-8'))
                
            if self.get_parameter('debug').value:
                self.get_logger().debug(f'Sent: {command.strip()}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to send motor commands: {e}')
            self.connected = False
    
    def cleanup(self):
        """Cleanup resources"""
        self.connected = False
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except:
                pass

def main(args=None):
    rclpy.init(args=args)
    
    try:
        arduino_driver = ArduinoDriver()
        rclpy.spin(arduino_driver)
    except KeyboardInterrupt:
        pass
    finally:
        if 'arduino_driver' in locals():
            arduino_driver.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()