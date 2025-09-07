from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Paths
    pkg_share = FindPackageShare('robot_navigation').find('robot_navigation')
    default_params_file = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file', default=default_params_file)
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use')
    
    # Include the Nav2 launch file
    nav2_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
        }.items()
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        
        # Launch Nav2
        nav2_launch,
    ])
