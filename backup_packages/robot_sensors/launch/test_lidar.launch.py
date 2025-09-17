from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
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
            'scan_mode': 'Standard',
            'serial_baudrate': 115200,
            'inverted': False,
            'angle_min': -3.14159,  # -180 degrees
            'angle_max': 3.14159,   # 180 degrees
        }],
        output='screen'
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
    ])
