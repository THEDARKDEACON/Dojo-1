from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('ros2arduino_bridge')
    
    # Create the launch description
    return LaunchDescription([
        # Arduino Bridge Node
        Node(
            package='ros2arduino_bridge',
            executable='arduino_bridge',
            name='arduino_bridge',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM0',  # Update this to match your Arduino port
                'baud_rate': 115200,
                'timeout': 1.0,
                'sensor_publish_rate': 10.0
            }],
            remappings=[
                ('/wheel_commands', '/cmd_vel'),  # Remap to match your control topic
                ('/wheel_encoders', '/odom'),     # Remap to match your odometry topic
                ('/imu/data', '/imu/data_raw'),    # Remap to match your IMU topic
            ]
        ),
        
        # Optional: Add a static transform publisher if needed
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_imu',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        # ),
    ])
