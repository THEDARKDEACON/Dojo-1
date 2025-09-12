#!/bin/bash

# Exit on error
set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Starting workspace fix script...${NC}"

# Define workspace directory
WORKSPACE_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SRC_DIR="${WORKSPACE_DIR}/src"

echo -e "${GREEN}Workspace directory: ${WORKSPACE_DIR}${NC}"

# Create required directories for all packages
create_directories() {
    echo -e "${YELLOW}Creating required directories...${NC}"
    
    # List of all packages
    local packages=(
        "robot_control"
        "robot_description"
        "robot_navigation"
        "robot_perception"
        "robot_sensors"
        "arduino_bridge"
        "ros2arduino_bridge"
        "robot_bringup"
    )
    
    for pkg in "${packages[@]}"; do
        echo "  - Setting up ${pkg}..."
        
        # Create standard directories
        mkdir -p "${SRC_DIR}/${pkg}/launch"
        mkdir -p "${SRC_DIR}/${pkg}/config"
        mkdir -p "${SRC_DIR}/${pkg}/resource"
        
        # For Python packages
        mkdir -p "${SRC_DIR}/${pkg}/src/${pkg}"
        touch "${SRC_DIR}/${pkg}/src/${pkg}/__init__.py"
        touch "${SRC_DIR}/${pkg}/resource/${pkg}"
        
        # For robot_description meshes
        if [ "${pkg}" == "robot_description" ]; then
            mkdir -p "${SRC_DIR}/${pkg}/meshes"
        fi
    done
}

# Fix CMake files
fix_cmake_files() {
    echo -e "${YELLOW}Fixing CMake files...${NC}"
    
    # Find and replace 'action(' with 'install(' in all CMakeLists.txt files
    find "${SRC_DIR}" -name "CMakeLists.txt" -type f -exec sed -i 's/action(/install(/g' {} \;
}

# Create navigation config files
create_nav_configs() {
    echo -e "${YELLOW}Creating navigation configuration files...${NC}"
    
    local nav_dir="${SRC_DIR}/robot_navigation/config"
    
    # Create map_server_params.yaml
    cat > "${nav_dir}/map_server_params.yaml" << 'EOL'
/**:
  ros__parameters:
    use_sim_time: False
    yaml_filename: map.yaml
    topic_name: map
    frame_id: map
EOL

    # Create nav2_params.yaml
    cat > "${nav_dir}/nav2_params.yaml" << 'EOL'
/**:
  ros__parameters:
    use_sim_time: False
    autostart: True
    bt_xml_filename: "navigate_w_replanning_and_recovery.xml"
    global_frame: map
    robot_base_frame: base_link
    robot_radius: 0.22
    transform_tolerance: 0.2
    planner_server:
      ros__parameters:
        expected_planner_frequency: 20.0
    controller_server:
      ros__parameters:
        controller_frequency: 20.0
EOL

    # Create other required config files with minimal content
    for file in "localization_params.yaml" "bt_navigator_params.yaml" "planner_params.yaml" \
                "controller_params.yaml" "costmap_common_params.yaml" \
                "global_costmap_params.yaml" "local_costmap_params.yaml"; do
        cat > "${nav_dir}/${file}" << 'EOL'
/**:
  ros__parameters:
    use_sim_time: False
EOL
    done
}

# Setup Python packages
setup_ros2arduino_bridge() {
    echo -e "${YELLOW}Setting up ros2arduino_bridge package...${NC}"
    
    local pkg_dir="${SRC_DIR}/ros2arduino_bridge"
    
    # Create required directories
    mkdir -p "${pkg_dir}/ros2arduino_bridge"
    mkdir -p "${pkg_dir}/launch"
    mkdir -p "${pkg_dir}/config"
    mkdir -p "${pkg_dir}/scripts"
    
    # Create __init__.py
    echo "from .arduino_bridge import ArduinoBridge" > "${pkg_dir}/ros2arduino_bridge/__init__.py"
    
    # Create arduino_bridge.py if it doesn't exist
    if [ ! -f "${pkg_dir}/ros2arduino_bridge/arduino_bridge.py" ]; then
        cat > "${pkg_dir}/ros2arduino_bridge/arduino_bridge.py" << 'EOL'
import rclpy
from rclpy.node import Node
import serial
import serial.tools.list_ports
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import time

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('debug', False)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('imu_frame_id', 'imu_link')
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.debug = self.get_parameter('debug').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        
        # Initialize serial connection
        self.serial_conn = None
        self.connect_arduino()
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        
        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Timer for reading from Arduino
        self.timer = self.create_timer(0.01, self.read_serial)  # 100Hz
        
        self.get_logger().info('Arduino Bridge Node Started')
    
    def connect_arduino(self):
        """Initialize serial connection to Arduino."""
        try:
            self.serial_conn = serial.Serial(self.port, self.baud_rate, timeout=1)
            self.get_logger().info(f'Connected to Arduino on {self.port}')
            # Give Arduino time to reset
            time.sleep(2)
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            raise
    
    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands."""
        if self.serial_conn and self.serial_conn.is_open:
            # Format: "vx,vy,vtheta\n"
            cmd_str = f"{msg.linear.x},{msg.linear.y},{msg.angular.z}\n"
            self.serial_conn.write(cmd_str.encode('utf-8'))
    
    def read_serial(self):
        """Read and process data from Arduino."""
        if not self.serial_conn or not self.serial_conn.is_open:
            self.get_logger().warn('Serial connection not active, attempting to reconnect...')
            self.connect_arduino()
            return
        
        try:
            if self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode('utf-8').strip()
                if self.debug:
                    self.get_logger().debug(f'Received: {line}')
                
                # Process the received data
                self.process_serial_data(line)
        except Exception as e:
            self.get_logger().error(f'Error reading from serial: {e}')
            self.serial_conn.close()
    
    def process_serial_data(self, data):
        """Process the data received from Arduino."""
        try:
            # Example format: "x,y,theta,vx,vy,omega,ax,ay,az,gx,gy,gz"
            parts = data.split(',')
            if len(parts) != 12:
                if self.debug:
                    self.get_logger().debug(f'Invalid data format: {data}')
                return
            
            # Parse odometry data
            x = float(parts[0])
            y = float(parts[1])
            theta = float(parts[2])
            vx = float(parts[3])
            vy = float(parts[4])
            omega = float(parts[5])
            
            # Parse IMU data
            ax = float(parts[6])
            ay = float(parts[7])
            az = float(parts[8])
            gx = float(parts[9])
            gy = float(parts[10])
            gz = float(parts[11])
            
            # Publish odometry
            self.publish_odometry(x, y, theta, vx, vy, omega)
            
            # Publish IMU data
            self.publish_imu(ax, ay, az, gx, gy, gz)
            
        except (ValueError, IndexError) as e:
            self.get_logger().error(f'Error parsing data: {e}, data: {data}')
    
    def publish_odometry(self, x, y, theta, vx, vy, omega):
        """Publish odometry message."""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.base_frame_id
        
        # Set position
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # Set velocity
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = omega
        
        # Publish
        self.odom_pub.publish(odom_msg)
    
    def publish_imu(self, ax, ay, az, gx, gy, gz):
        """Publish IMU message."""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.imu_frame_id
        
        # Set linear acceleration (convert from m/s^2 to g if needed)
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        
        # Set angular velocity (convert to rad/s if needed)
        imu_msg.angular_velocity.x = math.radians(gx)  # Convert from deg/s to rad/s
        imu_msg.angular_velocity.y = math.radians(gy)
        imu_msg.angular_velocity.z = math.radians(gz)
        
        # Publish
        self.imu_pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_conn and node.serial_conn.is_open:
            node.serial_conn.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOL
    fi
    
    # Create launch file if it doesn't exist
    if [ ! -f "${pkg_dir}/launch/ros2arduino_bridge.launch.py" ]; then
        cat > "${pkg_dir}/launch/ros2arduino_bridge.launch.py" << 'EOL'
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('ros2arduino_bridge')
    
    # Path to the parameters file
    params_file = os.path.join(pkg_dir, 'config', 'ros2arduino_bridge_params.yaml')
    
    # Create the launch description
    return LaunchDescription([
        # Arduino Bridge Node
        Node(
            package='ros2arduino_bridge',
            executable='arduino_bridge',
            name='arduino_bridge',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('/cmd_vel', '/cmd_vel'),
                ('/odom', '/odom'),
                ('/imu/data', '/imu/data'),
            ]
        )
    ])
EOL
    fi
    
    # Create config directory and params file if it doesn't exist
    mkdir -p "${pkg_dir}/config"
    if [ ! -f "${pkg_dir}/config/ros2arduino_bridge_params.yaml" ]; then
        cat > "${pkg_dir}/config/ros2arduino_bridge_params.yaml" << 'EOL'
/**:
  ros__parameters:
    # Serial port configuration
    port: /dev/ttyACM0
    baud_rate: 115200
    debug: false
    
    # Frame IDs
    odom_frame_id: 'odom'
    base_frame_id: 'base_link'
    imu_frame_id: 'imu_link'
    
    # Covariance values (placeholder values, adjust as needed)
    odom_pose_covariance_diagonal: [0.001, 0.001, 0.0, 0.0, 0.0, 0.1]
    odom_twist_covariance_diagonal: [0.01, 0.01, 0.0, 0.0, 0.0, 0.03]
    imu_orientation_covariance: [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
    imu_angular_velocity_covariance: [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02]
    imu_linear_acceleration_covariance: [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]
EOL
    fi
    
    # Create setup.py if it doesn't exist
    if [ ! -f "${pkg_dir}/setup.py" ]; then
        cat > "${pkg_dir}/setup.py" << 'EOL'
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ros2arduino_bridge'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            if '__pycache__' in path or filename.endswith('.pyc') or filename.endswith('~'):
                continue
            paths.append(os.path.join('..', path, filename))
    return paths

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=False,
    maintainer='robosync',
    maintainer_email='garethjoel77@gmail.com',
    description='ROS 2 Arduino Bridge for Dojo Robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_bridge = ros2arduino_bridge.arduino_bridge:main',
        ],
    },
)
EOL
    fi
    
    # Create package.xml if it doesn't exist
    if [ ! -f "${pkg_dir}/package.xml" ]; then
        cat > "${pkg_dir}/package.xml" << 'EOL'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ros2arduino_bridge</name>
  <version>0.1.0</version>
  <description>ROS 2 Arduino Bridge for Dojo Robot</description>
  <maintainer email="garethjoel77@gmail.com">Gareth Joel</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros_py</depend>
  <exec_depend>python3-pyserial</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOL
    fi
    
    # Create entry point script if it doesn't exist
    mkdir -p "${pkg_dir}/scripts"
    if [ ! -f "${pkg_dir}/scripts/arduino_bridge" ]; then
        cat > "${pkg_dir}/scripts/arduino_bridge" << 'EOL'
#!/usr/bin/env python3

from ros2arduino_bridge.arduino_bridge import main

if __name__ == '__main__':
    main()
EOL
        chmod +x "${pkg_dir}/scripts/arduino_bridge"
    fi
    
    echo -e "${GREEN}ros2arduino_bridge package setup complete!${NC}"
    
    local python_packages=(
        "robot_control"
        "robot_sensors"
        "arduino_bridge"
        "ros2arduino_bridge"
    )
    
    for pkg in "${python_packages[@]}"; do
        echo "  - Setting up ${pkg}..."
        
        # Create setup.py if it doesn't exist
        if [ ! -f "${SRC_DIR}/${pkg}/setup.py" ]; then
            cat > "${SRC_DIR}/${pkg}/setup.py" << EOL
from setuptools import find_packages, setup

package_name = '${pkg}'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
EOL
        fi
        
        # Create package.xml if it doesn't exist
        if [ ! -f "${SRC_DIR}/${pkg}/package.xml" ]; then
            cat > "${SRC_DIR}/${pkg}/package.xml" << EOL
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>${pkg}</name>
  <version>0.0.1</version>
  <description>TODO: Package description</description>
  <maintainer email="you@example.com">You</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOL
        fi
    done
}

# Main execution
main() {
    echo -e "${GREEN}Starting workspace fix process...${NC}"
    
    # Check if running in the correct directory
    if [ ! -d "${SRC_DIR}" ]; then
        echo -e "${YELLOW}Error: 'src' directory not found. Please run this script from your workspace root.${NC}"
        exit 1
    fi
    
    # Run all fix functions
    create_directories
    fix_cmake_files
    create_nav_configs
    setup_python_packages
    setup_ros2arduino_bridge
    
    echo -e "${GREEN}Workspace fix completed successfully!${NC}"
    echo -e "${YELLOW}You can now build the workspace with:${NC}"
    echo "  colcon build --symlink-install"
}

# Run main function
main
