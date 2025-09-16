from setuptools import setup, find_packages
import os
from glob import glob
from setuptools import find_namespace_packages

package_name = 'robot_sensors'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            if filename.endswith('.launch.py') or filename.endswith('.yaml') or filename.endswith('.rviz'):
                paths.append(os.path.join('..', path, filename))
    return paths

# Get all launch files
launch_files = glob('launch/*.launch.py')
launch_files = [os.path.join('launch', os.path.basename(f)) for f in launch_files]

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='.') + 
        find_namespace_packages(include=['camera_ros.*', 'nv21_converter_pkg.*', 'sllidar_ros2.*']),
    package_dir={
        '': '.',
        'camera_ros': 'camera_ros',
        'nv21_converter_pkg': 'nv21_converter_pkg',
        'sllidar_ros2': 'sllidar_ros2'
    },
    package_data={
        'camera_ros': ['resource/*', 'launch/*.launch.py', '**/*.py'],
        'nv21_converter_pkg': ['resource/*', 'launch/*.launch.py', '**/*.py'],
        'sllidar_ros2': ['launch/*.launch.py', 'rviz/*.rviz', 'config/*.yaml', '**/*.py']
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), launch_files),
    ],
    install_requires=[
        'setuptools>=58.2.0',
        'opencv-python>=4.5.0',
        'numpy>=1.19.5',
        'pyserial>=3.5',
        'picamera2>=0.3.0',
        'rclpy>=3.0.0',
        'sensor_msgs>=3.0.0',
        'cv_bridge_py>=3.0.0',
        'camera_info_manager_py>=0.2.0',
        'image_transport>=3.0.0',
        'libcamera>=0.0.1',
    ],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='Camera and LiDAR sensor interfaces for the robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = robot_sensors.nodes.camera_node:main',
        ],
    },
)
