from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'racecar_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
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
            'test = racecar_bringup.test:main',
            'cmd_vel_arbitration = racecar_bringup.cmd_vel_arbitration:main',
            'arduino_sensors = racecar_bringup.arduino_sensors:main',
        ],
    },
)
