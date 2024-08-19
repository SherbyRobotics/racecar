from setuptools import find_packages, setup

package_name = 'racecar_autopilot'

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
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slash_controller = racecar_autopilot.slash_controller:main',
            'wall_estimator = racecar_autopilot.wall_estimator:main',        
        ],
    },
)
