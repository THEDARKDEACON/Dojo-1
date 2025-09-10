from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision_system'

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Add launch and config files
data_files.extend([
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
])

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gareth Joel',
    maintainer_email='garethjoel77@gmail.com',
    description='Vision system for object detection and robot control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector = vision_system.object_detector:main',
        ],
    },
)
