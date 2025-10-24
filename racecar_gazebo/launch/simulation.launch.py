import os
from typing import Text

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context: LaunchContext, *args, **kwargs):
    # Package Directories
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    racecar_gazebo = get_package_share_directory("racecar_gazebo")

    gazeboDefaultResourcePath = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", "/opt/ros/jazzy/share"
    )
    addRacecarGazeboResourcePath = AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", racecar_gazebo)

    # Start Gazebo simulation
    __world_name = LaunchConfiguration("world").perform(context)
    assert isinstance(__world_name, Text), f"Failed to parse world_name: {__world_name}"
    world_name = os.path.join(racecar_gazebo, "worlds", f"racecar_{__world_name}.world")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r {world_name}"}.items(),
    )

    # Spawn a racecar in Gazebo
    spawn_racecar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(racecar_gazebo, "launch", "spawn_racecar.launch.py")]
        ),
    )

    return (gazeboDefaultResourcePath, addRacecarGazeboResourcePath, gazebo, spawn_racecar)


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="tunnel",
        description="Name of the world file to load into Gazebo",
        choices=["tunnel", "tunnel_genie", "circuit"],
    )

    return LaunchDescription([world_arg, OpaqueFunction(function=launch_setup)])
