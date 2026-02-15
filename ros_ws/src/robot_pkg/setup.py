import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'robot_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'msg'), glob('msg/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samurai Team',
    maintainer_email='samurai@robot.local',
    description='Samurai tracked robot with Adeept Robot HAT V3.1',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motor_node = robot_pkg.motor_node:main',
            'ultrasonic_node = robot_pkg.ultrasonic_node:main',
            'camera_node = robot_pkg.camera_node:main',
            'laser_node = robot_pkg.laser_node:main',
            'servo_node = robot_pkg.servo_node:main',
            'voice_node = robot_pkg.voice_node:main',
            'fsm_node = robot_pkg.fsm_node:main',
            'imu_node = robot_pkg.imu_node:main',
            'mqtt_bridge_node = robot_pkg.mqtt_bridge_node:main',
            'dashboard_node = robot_pkg.dashboard_node:main',
        ],
    },
)
