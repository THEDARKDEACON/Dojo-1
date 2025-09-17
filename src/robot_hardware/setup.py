from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robot_hardware'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, f'{package_name}.drivers'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial', 'numpy', 'opencv-python'],
    zip_safe=True,
    maintainer='Gareth Joel',
    maintainer_email='garethjoel77@gmail.com',
    description='Unified hardware interface for Dojo robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_driver = robot_hardware.drivers.arduino_driver:main',
            'camera_driver = robot_hardware.drivers.camera_driver:main',
            'lidar_driver = robot_hardware.drivers.lidar_driver:main',
            'hardware_manager = robot_hardware.hardware_manager:main',
        ],
    },
)