import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import AppendEnvironmentVariable

def generate_launch_description():
    # Package Directories    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    racecar_gazebo = get_package_share_directory('racecar_gazebo')

    world_name = os.path.join(get_package_share_directory('racecar_gazebo'), 'worlds', 'racecar_tunnel.world')
    models_path = os.path.join(get_package_share_directory('racecar_gazebo'), 'models')

    addEnvVariable = AppendEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH",models_path)
    print(models_path)

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
        addEnvVariable,
        gazebo,
        spawn,
    ])

if __name__ == '__main__':
    generate_launch_description()

