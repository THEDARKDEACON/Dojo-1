from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robot_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools', 
        'numpy>=1.19.0', 
        'opencv-python>=4.5.0',
        'rclpy',
        'sensor-msgs',
        'std-msgs',
        'geometry-msgs',
        'cv-bridge'
    ],
    zip_safe=False,
    maintainer='robosync',
    maintainer_email='robosync@example.com',
    description='Robot perception package for handling camera and vision processing',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_processor = robot_perception.camera_processor:main',
            'object_detector = robot_perception.object_detector:main',
        ],
    },
)
