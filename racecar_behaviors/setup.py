from setuptools import find_packages, setup
import os  
from glob import glob


package_name = 'racecar_behaviors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name,), glob('cfg/*.cfg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='lali3401@usherbrooke.ca',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blob_detector = racecar_behaviors.blob_detector:main',
            'goal_planner = racecar_behaviors.goal_planner:main',
            'labo_brushfire = racecar_behaviors.labo_brushfire:main',
            'laserscan_to_pointcloud = racecar_behaviors.laserscan_to_pointcloud:main',
            'libbehaviors = racecar_behaviors.libbehaviors:main',
            'obstacle_detector = racecar_behaviors.obstacle_detector:main',
            'path_following = racecar_behaviors.path_following:main',
            'u_turn = racecar_behaviors.u_turn:main',
            'u_turn_goal = racecar_behaviors.u_turn_goal:main',
        ],
    },
)
