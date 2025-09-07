#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_arduino = LaunchConfiguration('use_arduino', default='true')
    
    # Arduino bridge node
    arduino_bridge = Node(
        package='robot_control',
        executable='arduino_bridge',
        name='arduino_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            PathJoinSubstitution([
                FindPackageShare('robot_control'),
                'config',
                'arduino.yaml'
            ])
        ],
        condition=IfCondition(use_arduino)
    )
    
    # Cmd_vel to motor commands converter
    cmd_vel_to_motors = Node(
        package='robot_control',
        executable='cmd_vel_to_motors',
        name='cmd_vel_to_motors',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'wheel_separation': 0.3},  # Distance between wheels in meters
            {'wheel_radius': 0.05},     # Wheel radius in meters
            {'max_motor_speed': 255},   # Max PWM value for motors
            {'min_motor_speed': 50}     # Min PWM value to overcome friction
        ]
    )
    
    # Static transform publisher for base_link to base_footprint
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_footprint',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_footprint', 'base_link']
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
            
        DeclareLaunchArgument(
            'use_arduino',
            default_value='true',
            description='Enable Arduino bridge node'),
            
        arduino_bridge,
        cmd_vel_to_motors,
        static_tf,
    ])
