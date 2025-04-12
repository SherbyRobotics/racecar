from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


racecar_bringup_dir = get_package_share_directory("racecar_bringup")
rosbridge_server_dir = get_package_share_directory("rosbridge_server")

rosbridge_port_arg = DeclareLaunchArgument(
    "port", default_value="9090", description="Port for rosbridge websocket"
)

host_address_arg = DeclareLaunchArgument(
    "host_address", default_value="127.0.0.1", description="Address of the RaspberryPi"
)

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

camera_node = Node(
    package="v4l2_camera",
    executable="v4l2_camera_node",
    name="camera",
    parameters=[
        {
            "camera_frame_id": "racecar/camera_optical_link",
            "saturation": 100,
        }
    ],
    remappings=[
        ("image_raw", "racecar/camera"),
        ("camera_info", "racecar/camera_info"),
    ],
)

teleop_ld = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(f"{racecar_bringup_dir}/launch/teleop.launch.py"),
    launch_arguments={"serial_com": "True"}.items(),
)


def generate_launch_description():
    return LaunchDescription(
        [
            rosbridge_port_arg,
            host_address_arg,
            rosbridge_server_ld,
            web_video_server_node,
            camera_node,
            teleop_ld,
        ]
    )
