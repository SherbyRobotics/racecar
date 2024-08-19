from setuptools import find_packages, setup

package_name = 'racecar_beacon'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'poll_pos = racecar_beacon.lab_poll_pos:main',
            'ros_monitor = racecar_beacon.ros_monitor:main',
        ],
    },
)
