from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/root/Dojo/install/robot_description/share/robot_description/rviz/dojo_robot.rviz']
        )
    ])
