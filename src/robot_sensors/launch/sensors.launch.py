#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_camera = LaunchConfiguration('use_camera', default='true')
    use_lidar = LaunchConfiguration('use_lidar', default='true')
    
    # Camera parameters
    camera_name = LaunchConfiguration('camera_name', default='camera')
    camera_frame_id = LaunchConfiguration('camera_frame_id', default='camera_link')
    camera_info_url = LaunchConfiguration('camera_info_url', 
        default=f'package://robot_sensors/camera_ros/config/camera_info.yaml')
    
    # LiDAR parameters
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB0')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id', default='laser')
    lidar_angle_compensate = LaunchConfiguration('lidar_angle_compensate', default='true')
    lidar_scan_mode = LaunchConfiguration('lidar_scan_mode', default='Sensitivity')

    # Camera Node
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_sensors.camera_ros'), 'launch', 'camera.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'camera_name': camera_name,
            'camera_frame_id': camera_frame_id,
            'camera_info_url': camera_info_url,
            'fps': '30.0',
            'width': '640',
            'height': '480'
        }.items(),
        condition=IfCondition(use_camera)
    )

    # LiDAR Node using sllidar_ros2
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_sensors.sllidar_ros2'), 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': lidar_port,
            'frame_id': lidar_frame_id,
            'angle_compensate': lidar_angle_compensate,
            'scan_mode': lidar_scan_mode
        }.items(),
        condition=IfCondition(use_lidar)
    )

    # Define launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
        
    declare_use_camera = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Enable camera node')
        
    declare_use_lidar = DeclareLaunchArgument(
        'use_lidar',
        default_value='true',
        description='Enable LiDAR node')
        
    declare_camera_name = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Name of the camera')
        
    declare_camera_frame_id = DeclareLaunchArgument(
        'camera_frame_id',
        default_value='camera_link',
        description='TF frame ID for the camera')
        
    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the LiDAR sensor')
        
    declare_lidar_frame_id = DeclareLaunchArgument(
        'lidar_frame_id',
        default_value='laser',
        description='TF frame ID for the LiDAR')

    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_camera)
    ld.add_action(declare_use_lidar)
    ld.add_action(declare_camera_name)
    ld.add_action(declare_camera_frame_id)
    ld.add_action(declare_lidar_port)
    ld.add_action(declare_lidar_frame_id)
    
    # Add nodes
    ld.add_action(camera_launch)
    ld.add_action(lidar_launch)
    
    return ld
