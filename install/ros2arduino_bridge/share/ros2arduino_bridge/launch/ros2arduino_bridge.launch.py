import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    # Serial port configuration
    declared_arguments.append(
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyACM0',
            description='Serial port for Arduino communication',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'baud',
            default_value='115200',
            description='Baud rate for serial communication',
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable debug output',
        )
    )
    
    # Get launch arguments
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    debug = LaunchConfiguration('debug')
    
    # Path to the parameters file
    params_file = os.path.join(
        get_package_share_directory('ros2arduino_bridge'),
        'config',
        'ros2arduino_bridge_params.yaml'
    )
    
    # Main bridge node
    bridge_node = Node(
        package='ros2arduino_bridge',
        executable='arduino_bridge',
        name='arduino_bridge',
        output='screen',
        parameters=[
            params_file,
            {
                'port': port,
                'baud_rate': baud,
                'debug': debug
            }
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/odom'),
            ('/imu/data', '/imu/data'),
        ]
    )
    
    # Create launch description with declared arguments
    return LaunchDescription(declared_arguments + [
        bridge_node,
    ])
