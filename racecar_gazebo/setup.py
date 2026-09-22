from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'racecar_gazebo'

data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]


def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:
        
        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                    
                else:
                    paths_dict[install_path] = [file_path]
                
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=package_files(data_files,['models/', 'launch/', 'worlds/','config/', 'meshes/']),
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
