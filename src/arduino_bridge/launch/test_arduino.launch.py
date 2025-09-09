from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('arduino_bridge')
    
    # Arduino bridge node
    arduino_bridge_node = Node(
        package='arduino_bridge',
        executable='arduino_bridge_node',
        name='arduino_bridge',
        output='screen',
        parameters=[{
            'port': '/dev/ttyACM0',  # Update this to match your Arduino port
            'baud_rate': 115200,
            'wheel_separation': 0.3,  # meters
            'wheel_radius': 0.05,     # meters
            'max_motor_speed': 255    # 0-255
        }]
    )
    
    # Teleop twist keyboard node for testing
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',  # Run in a new terminal window
        remappings=[
            ('/cmd_vel', '/cmd_vel')
        ]
    )
    
    return LaunchDescription([
        arduino_bridge_node,
        teleop_node
    ])
