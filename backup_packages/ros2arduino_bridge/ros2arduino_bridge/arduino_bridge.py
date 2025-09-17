
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import serial
import time

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('debug', False)
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.debug = self.get_parameter('debug').value
        
        # Initialize serial connection
        self.serial_conn = None
        self.connect_serial()
        
        # Setup publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        
        # Setup subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Setup timer for reading from serial
        self.timer = self.create_timer(0.01, self.read_serial)  # 100Hz
        
        self.get_logger().info('Arduino Bridge Node Started')
    
    def connect_serial(self):
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=1.0
            )
            time.sleep(2)  # Wait for Arduino to reset
            self.get_logger().info(f'Connected to Arduino on {self.port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {str(e)}')
            raise
    
    def cmd_vel_callback(self, msg):
        # Send velocity commands to Arduino
        # Format: 'v,linear_x,angular_z\n'
        if self.serial_conn and self.serial_conn.is_open:
            try:
                cmd_str = f'v,{msg.linear.x},{msg.angular.z}\n'
                if self.debug:
                    self.get_logger().info(f'Sending: {cmd_str.strip()}')
                
                self.serial_conn.write(cmd_str.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f'Error sending command: {str(e)}')
    
    def read_serial(self):
        if self.serial_conn and self.serial_conn.is_open:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    if self.debug:
                        self.get_logger().info(f'Received: {line}')
                    
                    # Process the received data
                    self.process_serial_data(line)
            except Exception as e:
                self.get_logger().error(f'Error reading serial: {str(e)}')
    
    def process_serial_data(self, data):
        # Process the data received from Arduino
        # Example format: 'o,x,y,theta,vx,wz' for odometry
        # or 'i,ax,ay,az,gx,gy,gz' for IMU
        try:
            parts = data.split(',')
            if len(parts) < 2:
                return
                
            data_type = parts[0]
            
            if data_type == 'o':  # Odometry data
                if len(parts) >= 6:
                    odom_msg = Odometry()
                    odom_msg.header.stamp = self.get_clock().now().to_msg()
                    odom_msg.header.frame_id = 'odom'
                    odom_msg.child_frame_id = 'base_link'
                    
                    # Set position
                    odom_msg.pose.pose.position.x = float(parts[1])
                    odom_msg.pose.pose.position.y = float(parts[2])
                    odom_msg.pose.pose.position.z = 0.0
                    
                    # Set orientation (convert yaw to quaternion)
                    yaw = float(parts[3])
                    odom_msg.pose.pose.orientation.z = yaw
                    
                    # Set velocities
                    odom_msg.twist.twist.linear.x = float(parts[4])
                    odom_msg.twist.twist.angular.z = float(parts[5])
                    
                    self.odom_pub.publish(odom_msg)
            
            elif data_type == 'i':  # IMU data
                if len(parts) >= 7:
                    imu_msg = Imu()
                    imu_msg.header.stamp = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = 'imu_link'
                    
                    # Set linear acceleration (convert from g to m/s^2 if needed)
                    imu_msg.linear_acceleration.x = float(parts[1]) * 9.81
                    imu_msg.linear_acceleration.y = float(parts[2]) * 9.81
                    imu_msg.linear_acceleration.z = float(parts[3]) * 9.81
                    
                    # Set angular velocity (convert from deg/s to rad/s)
                    imu_msg.angular_velocity.x = float(parts[4]) * 3.14159 / 180.0
                    imu_msg.angular_velocity.y = float(parts[5]) * 3.14159 / 180.0
                    imu_msg.angular_velocity.z = float(parts[6]) * 3.14159 / 180.0
                    
                    self.imu_pub.publish(imu_msg)
        
        except Exception as e:
            self.get_logger().error(f'Error processing serial data: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ArduinoBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in Arduino Bridge: {str(e)}')
    finally:
        if node.serial_conn and node.serial_conn.is_open:
            node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
