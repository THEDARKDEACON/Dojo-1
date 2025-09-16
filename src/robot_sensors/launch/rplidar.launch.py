from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Default parameters
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    
    # Include the RPLIDAR launch file
    rplidar_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('rplidar_ros'),
            'launch',
            'rplidar_a2m8.launch',
        ]),
        launch_arguments={
            'serial_port': serial_port,
            'frame_id': frame_id,
        }.items()
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
        
        # Include RPLIDAR launch
        rplidar_launch,
    ])
