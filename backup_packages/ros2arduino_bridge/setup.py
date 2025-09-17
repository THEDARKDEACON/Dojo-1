from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ros2arduino_bridge'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            if '__pycache__' in path or filename.endswith('.pyc') or filename.endswith('~'):
                continue
            paths.append(os.path.join('..', path, filename))
    return paths

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/ros2arduino_bridge.launch.py')),  # Updated this line
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=False,
    maintainer='robosync',
    maintainer_email='garethjoel77@gmail.com',
    description='ROS 2 Arduino Bridge for Dojo Robot',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'arduino_bridge = ros2arduino_bridge.arduino_bridge:main',
        ],
    },
)
