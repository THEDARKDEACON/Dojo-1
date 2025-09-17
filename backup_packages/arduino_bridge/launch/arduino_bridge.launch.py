from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('arduino_bridge')
    
    # Define the node
    arduino_bridge_node = Node(
        package='arduino_bridge',
        executable='arduino_bridge_node',
        name='arduino_bridge_node',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'arduino_bridge_params.yaml')]
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add nodes to the launch description
    ld.add_action(arduino_bridge_node)
    
    return ld
