#!/usr/bin/env python3
"""
Improved Robot Bringup Launch File
Uses new modular hardware architecture
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch arguments
    use_hardware = LaunchConfiguration('use_hardware', default='true')
    use_control = LaunchConfiguration('use_control', default='true')
    use_perception = LaunchConfiguration('use_perception', default='false')
    use_navigation = LaunchConfiguration('use_navigation', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Hardware layer - new unified hardware interface
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_hardware'), 'launch', 'hardware.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_hardware)
    )
    
    # Control layer - high-level control coordination
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_control'), 'launch', 'control.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_control)
    )
    
    # Perception layer - optional computer vision and AI
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_perception'), 'launch', 'perception.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_perception)
    )
    
    # Navigation layer - optional autonomous navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_navigation'), 'launch', 'nav2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_navigation)
    )
    
    # Group all launches for better organization
    robot_group = GroupAction([
        hardware_launch,
        control_launch,
        perception_launch,
        navigation_launch
    ])
    
    # Launch argument declarations
    declares = [
        DeclareLaunchArgument('use_hardware', default_value='true', 
                            description='Launch hardware drivers'),
        DeclareLaunchArgument('use_control', default_value='true',
                            description='Launch control system'),
        DeclareLaunchArgument('use_perception', default_value='false',
                            description='Launch perception system'),
        DeclareLaunchArgument('use_navigation', default_value='false',
                            description='Launch navigation system'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                            description='Use simulation clock'),
    ]

    return LaunchDescription([
        *declares,
        robot_group
    ])
