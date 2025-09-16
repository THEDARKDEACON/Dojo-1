#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_camera = LaunchConfiguration('use_camera', default='true')
    use_lidar = LaunchConfiguration('use_lidar', default='true')

    # Camera Node
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_sensors'), 'launch', 'camera.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_camera)
    )

    # LiDAR Node using sllidar_ros2
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser',
            'angle_compensate': 'true',
            'scan_mode': 'Sensitivity'
        }.items(),
        condition=IfCondition(use_lidar)
    )

    # Create launch description
    ld = LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='false',
                            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('use_camera', default_value='true',
                            description='Enable camera node'),
        DeclareLaunchArgument('use_lidar', default_value='true',
                            description='Enable LiDAR node'),
    ])

    # Add nodes to launch description
    ld.add_action(camera_launch)
    ld.add_action(lidar_launch)
    
    return ld
