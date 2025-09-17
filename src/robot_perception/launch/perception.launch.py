from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    enable_vision = LaunchConfiguration('enable_vision', default='true')
    
    # Camera processing node
    camera_processor = Node(
        package='robot_perception',
        executable='camera_processor',
        name='camera_processor',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'camera_topic': 'image_raw'},
            {'camera_info_topic': 'camera_info'},
            {'debug': True}
        ],
        condition=IfCondition(enable_vision)
    )
    
    # Object detection node (placeholder for your model)
    object_detector = Node(
        package='robot_perception',
        executable='object_detector',
        name='object_detector',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'model_path': ''},  # Path to your model
            {'confidence_threshold': 0.5}
        ],
        condition=IfCondition(enable_vision)
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
            
        DeclareLaunchArgument(
            'enable_vision',
            default_value='true',
            description='Enable vision processing nodes'),
        
        camera_processor,
        # object_detector,  # Uncomment when you have a model
    ])
