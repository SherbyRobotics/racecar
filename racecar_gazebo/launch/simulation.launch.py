import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Package Directories
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    racecar_gazebo = get_package_share_directory("racecar_gazebo")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="tunnel",
        description="Name of the world file to load into Gazebo",
        choices=["tunnel", "tunnel_genie", "circuit"],
    )

    gazeboDefaultResourcePath = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", "/opt/ros/jazzy/share"
    )
    addRacecarGazeboResourcePath = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", racecar_gazebo
    )

    # Inside generate_launch_description() function
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [
                "-r ",
                PathJoinSubstitution(
                    [
                        racecar_gazebo,
                        "worlds",
                        ["racecar_", LaunchConfiguration("world"), ".world"],
                    ]
                ),
            ]
        }.items(),
    )  # TODO: Find a better way to use the `world` argument at launch.

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
