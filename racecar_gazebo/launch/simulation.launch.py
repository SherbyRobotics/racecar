import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Package Directories
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    racecar_gazebo = get_package_share_directory("racecar_gazebo")

    world_name = os.path.join(
        racecar_gazebo, "worlds", f"racecar_{LaunchConfiguration('world')}.world"
    )

    gazeboDefaultResourcePath = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", "/opt/ros/jazzy/share"
    )
    addRacecarGazeboResourcePath = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", racecar_gazebo
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="tunnel",
        description="The name of the world file to load",
        choices=["tunnel", "tunnel_genie", "circuit"],
    )

    # Inside generate_launch_description() function
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": f"-r {world_name}"}.items(),
    )

    # Include spawn launch file
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(racecar_gazebo, "launch", "spawn_racecar.launch.py")]
        ),
    )

    return LaunchDescription(
        [
            world_arg,
            gazeboDefaultResourcePath,
            addRacecarGazeboResourcePath,
            gazebo,
            spawn,
        ]
    )
