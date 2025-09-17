#!/usr/bin/env python3
"""
Hardware Launch File for Dojo Robot
Launches all hardware drivers in a coordinated manner
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Launch arguments
    use_arduino = LaunchConfiguration('use_arduino', default='true')
    use_camera = LaunchConfiguration('use_camera', default='true') 
    use_lidar = LaunchConfiguration('use_lidar', default='true')
    use_hardware_manager = LaunchConfiguration('use_hardware_manager', default='true')
    config_file = LaunchConfiguration('config_file', default='hardware.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Get config file path
    config_path = PathJoinSubstitution([
        FindPackageShare('robot_hardware'),
        'config',
        config_file
    ])
    
    # Arduino Driver Node
    arduino_node = Node(
        package='robot_hardware',
        executable='arduino_driver',
        name='arduino_driver',
        parameters=[config_path, {'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_arduino)
    )
    
    # Camera Driver Node
    camera_node = Node(
        package='robot_hardware',
        executable='camera_driver',
        name='camera_driver',
        parameters=[config_path, {'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_camera)
    )
    
    # LiDAR Driver Node
    lidar_node = Node(
        package='robot_hardware',
        executable='lidar_driver',
        name='lidar_driver',
        parameters=[config_path, {'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_lidar)
    )
    
    # Hardware Manager Node
    hardware_manager_node = Node(
        package='robot_hardware',
        executable='hardware_manager',
        name='hardware_manager',
        parameters=[config_path, {'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_hardware_manager)
    )
    
    # Group all hardware nodes
    hardware_group = GroupAction([
        arduino_node,
        camera_node,
        lidar_node,
        hardware_manager_node
    ])
    
    # Launch arguments declarations
    declare_use_arduino = DeclareLaunchArgument(
        'use_arduino',
        default_value='true',
        description='Enable Arduino driver')
        
    declare_use_camera = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Enable camera driver')
        
    declare_use_lidar = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Enable LiDAR driver')
        
    declare_use_hardware_manager = DeclareLaunchArgument(
        'use_hardware_manager',
        default_value='true',
        description='Enable hardware manager')
        
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value='hardware.yaml',
        description='Hardware configuration file')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    return LaunchDescription([
        declare_use_arduino,
        declare_use_camera,
        declare_use_lidar,
        declare_use_hardware_manager,
        declare_config_file,
        declare_use_sim_time,
        hardware_group
    ])