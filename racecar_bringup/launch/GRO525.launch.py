import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

racecar_bringup_dir = get_package_share_directory("racecar_bringup")
rosbridge_server_dir = get_package_share_directory("rosbridge_server")

rosbridge_server_ld = IncludeLaunchDescription(
    XMLLaunchDescriptionSource(
        f"{rosbridge_server_dir}/launch/rosbridge_websocket_launch.xml"
    ),
    launch_arguments={"port": LaunchConfiguration("port")}.items(),
)

web_video_server_node = Node(
    package="web_video_server",
    executable="web_video_server",
    name="web_video_server",
    output="screen",
    parameters=[{"address": LaunchConfiguration("host_address")}],
)

def generate_launch_description():
    return LaunchDescription([
    DeclareLaunchArgument("host_address", 
                        default_value="10.42.0.1", 
                        description="Address of the Raspberry Pi"),
    DeclareLaunchArgument("port", 
                        default_value="9090", 
                        description="Port for rosbridge websocket"),
        rosbridge_server_ld,
        web_video_server_node,
])
