from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # File paths
    pkg_share = FindPackageShare('robot_description').find('robot_description')
    default_rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'robot.rviz'])
    default_urdf = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_config = LaunchConfiguration('rviz_config', default=default_rviz_config)
    urdf = LaunchConfiguration('urdf', default=default_urdf)
    
    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to RViz config file'
    )
    
    declare_urdf = DeclareLaunchArgument(
        'urdf',
        default_value=default_urdf,
        description='Path to URDF file'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf])
        }]
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz_config,
        declare_urdf,
        
        # Start robot state publisher
        robot_state_publisher,
        
        # Start joint state publisher
        joint_state_publisher,
        
        # Start RViz
        rviz,
    ])
