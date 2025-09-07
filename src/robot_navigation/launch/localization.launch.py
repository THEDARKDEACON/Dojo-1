from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Paths
    pkg_share = FindPackageShare('robot_navigation').find('robot_navigation')
    default_params_file = PathJoinSubstitution([pkg_share, 'config', 'localization_params.yaml'])
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file', default=default_params_file)
    map_file = LaunchConfiguration('map', default=PathJoinSubstitution([pkg_share, 'maps', 'map.yaml']))
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the AMCL parameters file')
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([pkg_share, 'maps', 'map.yaml']),
        description='Full path to map yaml file')
    
    # AMCL node
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
        remappings=[('scan', 'scan')]  # Make sure this matches your LiDAR topic
    )
    
    # Lifecycle manager for AMCL
    lifecycle_nodes = ['amcl']
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': lifecycle_nodes}
        ]
    )
    
    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_file}
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_map,
        
        # Start nodes
        map_server,
        amcl,
        lifecycle_manager,
    ])
