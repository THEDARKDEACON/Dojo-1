from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Paths
    pkg_share = FindPackageShare('robot_navigation').find('robot_navigation')
    default_map_file = PathJoinSubstitution([pkg_share, 'maps', 'map.yaml'])
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map', default=default_map_file)
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file')
    
    # Map server node
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
    
    # Lifecycle manager for map server
    lifecycle_nodes = ['map_server']
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': lifecycle_nodes}
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        
        # Start nodes
        map_server,
        lifecycle_manager,
    ])
