#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_camera = LaunchConfiguration('use_camera', default='true')
    use_lidar = LaunchConfiguration('use_lidar', default='true')

    camera_node = Node(
        package='robot_sensors',
        executable='camera_publisher',
        name='camera_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_camera)
    )

    # Example: Placeholder for a LiDAR driver (e.g., RPLidar A1M8 or similar)
    # Replace with your actual LiDAR driver package
    lidar_driver_launch = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'rplidar_ros', 'rplidar_a1_launch.py',
            'serial_port:=/dev/ttyUSB0',
            'frame_id:=laser',
            'angle_compensate:=true'
        ],
        output='screen',
        condition=IfCondition(use_lidar)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                             description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('use_camera', default_value='true',
                             description='Enable camera node'),
        DeclareLaunchArgument('use_lidar', default_value='true',
                             description='Enable LiDAR node'),
        camera_node,
        # lidar_driver_launch,  # Uncomment when you have a LiDAR driver
    ])
