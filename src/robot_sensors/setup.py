from setuptools import setup

package_name = 'robot_sensors'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sensors.launch.py'])
    ],
    install_requires=['setuptools', 'opencv-python'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='Camera and LiDAR sensor interfaces for the robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = robot_sensors.camera_publisher:main',
        ],
    },
)
