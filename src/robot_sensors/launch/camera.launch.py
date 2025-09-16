from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Define launch arguments
    camera_name = LaunchConfiguration('camera_name', default='camera')
    frame_id = LaunchConfiguration('frame_id', default='camera_link')
    width = LaunchConfiguration('width', default='640')
    height = LaunchConfiguration('height', default='480')
    fps = LaunchConfiguration('fps', default='30.0')
    
    # Camera node
    camera_node = Node(
        package='robot_sensors',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'camera_name': camera_name,
            'frame_id': frame_id,
            'width': width,
            'height': height,
            'fps': fps,
        }]
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('camera_name', default_value=camera_name,
                            description='Name of the camera'),
        DeclareLaunchArgument('frame_id', default_value=frame_id,
                            description='TF frame ID for the camera'),
        DeclareLaunchArgument('width', default_value=width,
                            description='Image width'),
        DeclareLaunchArgument('height', default_value=height,
                            description='Image height'),
        DeclareLaunchArgument('fps', default_value=fps,
                            description='Frames per second'),
        
        # Nodes
        camera_node,
    ])
