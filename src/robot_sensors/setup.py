from setuptools import setup
import os
from glob import glob

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
    packages=[package_name, f'{package_name}.nodes'],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), launch_files),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy', 'pyserial'],
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
