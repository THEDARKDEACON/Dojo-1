from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Default parameters
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    
    # RPLIDAR Node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': serial_port,
            'frame_id': frame_id,
            'angle_compensate': True,
            'scan_mode': 'Standard',  # Can be 'Standard', 'Express', or 'Boost'
            'serial_baudrate': 115200,  # A1M8 uses 115200 baud rate
            'inverted': False,
            'angle_min': -3.14159,  # -180 degrees
            'angle_max': 3.14159,   # 180 degrees
        }],
        output='screen'
    )
    
    # Laser filter to remove noise
    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory('robot_sensors'),
                'config',
                'laser_filter_config.yaml'
            ])
        ],
        remappings=[
            ('scan', 'scan_raw'),
            ('scan_filtered', 'scan')
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Serial port for RPLIDAR',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='TF frame ID for the LiDAR',
        ),
        
        # Start RPLIDAR node
        rplidar_node,
        
        # Start laser filter after a short delay
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=rplidar_node,
                on_exit=[laser_filter_node],
            )
        ),
    ])
