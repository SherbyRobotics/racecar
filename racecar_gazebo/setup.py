from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'racecar_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/includes/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='clocal',
    maintainer_email='lali3401@usherbrooke.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_ackermann_drive = racecar_gazebo.cmd_vel_to_ackermann_drive:main',
            'servo_commands = racecar_gazebo.servo_commands:servo_commands',
            'gazebo_odometry = racecar_gazebo.gazebo_odometry:main'
        ],
    },
)
