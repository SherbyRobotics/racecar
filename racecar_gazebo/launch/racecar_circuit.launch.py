import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Package Directories    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    racecar_gazebo = get_package_share_directory('racecar_gazebo')

    world_name = os.path.join(get_package_share_directory('racecar_gazebo'), 'worlds', 'racecar_circuit.world')

    # Inside generate_launch_description() function
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_name}'}.items(),
    )

    # Include spawn launch file
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(racecar_gazebo, 'launch', 'spawn_racecar.launch.py')]),
    )

    return LaunchDescription([
        gazebo,
        spawn,
    ])