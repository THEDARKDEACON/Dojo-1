from setuptools import setup
import os
from glob import glob

package_name = 'arduino_bridge'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join('..', path, filename))
    return paths

# Get launch and config files
launch_files = package_files(os.path.join(package_name, 'launch'))
config_files = package_files(os.path.join(package_name, 'config'))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include config files
        (os.path.join('share', package_name, 'config'), 
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools', 'pyserial', 'numpy'],
    zip_safe=True,
    maintainer='thedarkdeacon',
    maintainer_email='garethjoel77@gmail.com',
    description='ROS 2 package for communicating with Arduino over serial',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_bridge_node = arduino_bridge.arduino_bridge_node:main',
            'test_arduino_bridge = arduino_bridge.test_arduino_bridge:main',
        ],
    },
)
