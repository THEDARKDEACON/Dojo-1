import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Get the package directory
    pkg_robot_gazebo = get_package_share_directory('robot_gazebo')
    pkg_robot_description = get_package_share_directory('robot_description')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless = LaunchConfiguration('headless', default='false')
    world = LaunchConfiguration('world', default='empty.world')
    
    # Path to the world file
    world_path = os.path.join(pkg_robot_gazebo, 'worlds', world)
    
    # Path to the RViz configuration file
    rviz_config_path = os.path.join(pkg_robot_gazebo, 'rviz', 'robot_simulation.rviz')
    
    # Robot description
    robot_description = Command(['xacro ', os.path.join(pkg_robot_description, 'urdf', 'robot.urdf.xacro')])
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'false',
            'pause': 'false',
            'world': world_path,
            'gui': 'true',
            'headless': headless,
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
            'publish_frequency': 50.0
        }]
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'robot'],
        output='screen'
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz', default='true'))
    )
    
    # Joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager']
    )
    
    # Robot controller
    robot_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager']
    )
    
    # Delay start of robot controller after joint state broadcaster
    delay_robot_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[robot_controller],
        )
    )
    
    # Delay start of RViz after robot is spawned
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[rviz],
        )
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo in headless mode if true'),
            
        DeclareLaunchArgument(
            'world',
            default_value='empty.world',
            description='World file to load'),
            
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz if true'),
        
        # Nodes
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster,
        delay_robot_controller,
        delay_rviz,
    ])
