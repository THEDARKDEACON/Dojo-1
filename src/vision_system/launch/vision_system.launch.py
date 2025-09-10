from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('vision_system')
    
    # Path to parameter file
    params_file = os.path.join(pkg_dir, 'config', 'object_detector_params.yaml')
    
    # Object Detector Node
    object_detector_node = Node(
        package='vision_system',
        executable='object_detector',
        name='object_detector',
        output='screen',
        parameters=[params_file],
        # Enable better performance on multi-core systems
        prefix=['taskset -c 0,1'],  # Pin to specific CPU cores
        # Uncomment for GPU support
        # environment={'CUDA_VISIBLE_DEVICES': '0'},
    )
    
    # TODO: Add camera node when hardware is available
    # camera_node = Node(
    #     package='usb_cam',
    #     executable='usb_cam_node_exe',
    #     name='usb_cam',
    #     parameters=[{
    #         'video_device': '/dev/video0',
    #         'image_width': 640,
    #         'image_height': 480,
    #         'framerate': 30.0,
    #     }]
    # )
    
    return LaunchDescription([
        object_detector_node,
        # camera_node,
    ])
