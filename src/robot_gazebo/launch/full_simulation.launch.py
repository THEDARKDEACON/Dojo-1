#!/usr/bin/env python3
"""
Complete Dojo Robot Simulation with Full Sensor Suite
Includes: Robot Motion + LiDAR Mapping + Camera Feed + RViz Visualization
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    world_name = LaunchConfiguration('world', default='dojo_world.world')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    rviz = LaunchConfiguration('rviz', default='true')
    use_teleop = LaunchConfiguration('use_teleop', default='true')
    slam = LaunchConfiguration('slam', default='true')
    
    # === GAZEBO SIMULATION ===
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_gazebo'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_name,
            'gui': gui,
            'use_sim_time': use_sim_time,
            'rviz': 'false'  # We'll launch RViz separately with full config
        }.items()
    )
    
    # === ROBOT CONTROL SYSTEM ===
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_control'),
                'launch',
                'control.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_legacy_nodes': 'false',
            'use_control_manager': 'true'
        }.items()
    )
    
    # === SLAM TOOLBOX (for mapping) ===
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('robot_gazebo'),
                'config',
                'slam_config.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(slam)
    )
    
    # === TELEOP KEYBOARD CONTROL ===
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', 'cmd_vel_manual')  # Connect to control manager
        ],
        condition=IfCondition(use_teleop),
        prefix='xterm -e'  # Run in separate terminal
    )
    
    # === IMAGE PROCESSING (for camera visualization) ===
    image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='camera_viewer',
        arguments=['/image_raw'],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('camera_view', default='false'))
    )
    
    # === RVIZ WITH FULL SIMULATION CONFIG ===
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        'rviz',
        'full_simulation.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(rviz)
    )
    
    # === ROBOT LOCALIZATION (for better odometry) ===
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('robot_gazebo'),
                'config',
                'ekf_config.yaml'
            ]),
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(LaunchConfiguration('use_ekf', default='true'))
    )
    
    # === MAP SERVER (optional - for pre-made maps) ===
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': PathJoinSubstitution([
                FindPackageShare('robot_gazebo'),
                'maps',
                'dojo_map.yaml'
            ])},
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(LaunchConfiguration('use_map_server', default='false'))
    )
    
    # === DELAYED NODES (start after Gazebo is ready) ===
    delayed_slam = TimerAction(
        period=3.0,  # Wait 3 seconds for Gazebo to start
        actions=[slam_toolbox_node]
    )
    
    delayed_rviz = TimerAction(
        period=2.0,  # Wait 2 seconds
        actions=[rviz_node]
    )
    
    # === GROUP ALL SIMULATION COMPONENTS ===
    simulation_group = GroupAction([
        gazebo_launch,
        control_launch,
        delayed_slam,
        delayed_rviz,
        teleop_node,
        robot_localization_node,
        map_server_node,
        image_view_node
    ])
    
    return LaunchDescription([
        # === LAUNCH ARGUMENTS ===
        DeclareLaunchArgument('world', default_value='dojo_world.world',
                            description='Gazebo world file (empty.world, dojo_world.world)'),
        DeclareLaunchArgument('gui', default_value='true',
                            description='Start Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true',
                            description='Start RViz with full simulation config'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                            description='Use simulation clock'),
        DeclareLaunchArgument('use_teleop', default_value='true',
                            description='Start keyboard teleop control'),
        DeclareLaunchArgument('slam', default_value='true',
                            description='Start SLAM for mapping'),
        DeclareLaunchArgument('use_ekf', default_value='true',
                            description='Use Extended Kalman Filter for localization'),
        DeclareLaunchArgument('use_map_server', default_value='false',
                            description='Start map server with pre-made map'),
        DeclareLaunchArgument('camera_view', default_value='false',
                            description='Open separate camera viewer window'),
        
        # === LAUNCH SIMULATION ===
        simulation_group
    ])