from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'nv21_converter_pkg'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            if filename.endswith('.launch.py') or filename.endswith('.yaml') or filename.endswith('.rviz'):
                paths.append(os.path.join('..', path, filename))
    return paths

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='.'),
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'cv_bridge'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='NV21 to RGB/BGR converter for ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nv21_to_rgb = nv21_converter_pkg.nv21_to_rgb:main',
        ],
    },
)
