from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'ros2arduino_bridge'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            # Skip Python cache and other unnecessary files
            if '__pycache__' in path or filename.endswith('.pyc') or filename.endswith('~'):
                continue
            paths.append(os.path.join('..', path, filename))
    return paths

# Get all necessary files
launch_files = package_files('launch')
config_files = package_files('config')
arduino_files = package_files('arduino')

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include config files
        (os.path.join('share', package_name, 'config'), 
         glob(os.path.join('config', '*.*'))),
        # Include Arduino files
        (os.path.join('share', package_name, 'arduino'), 
         [f for f in glob(os.path.join('arduino', '**', '*'), recursive=True) 
          if os.path.isfile(f) and not f.endswith('.pyc')]),
    ],
    install_requires=['setuptools', 'pyserial>=3.5'],
    zip_safe=False,  # Required for proper Python package loading
    maintainer='thedarkdeacon',
    maintainer_email='garethjoel77@gmail.com',
    description='ROS 2 bridge for Arduino motor control and sensor reading',
    license='Apache-2.0',
    tests_require=['pytest', 'pytest-cov', 'pytest-mock'],
    entry_points={
        'console_scripts': [
            'arduino_bridge = ros2arduino_bridge.arduino_bridge:main',
            'test_arduino_bridge = ros2arduino_bridge.test_arduino_bridge:main',
            'arduino_bridge_tester = test_arduino_bridge:main',
        ],
    },
)
