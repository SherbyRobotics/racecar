from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Minimalist launch file for the racecar's teleoperation.
    """
    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy",
                parameters=[{"deadzone": 0.05}],
                arguments=["dev", "/dev/input/js0"],
                output="screen",
            ),
            Node(
                package="racecar_teleop",
                executable="slash_teleop",
                name="teleop",
                output="screen",
            ),
            Node(
                package="racecar_autopilot",
                executable="slash_controller",
                name="controller",
                output="screen",
            ),
            Node(
                package="pb2roscpp",
                executable="pb2roscpp",
                name="arduino",
                output="screen",
            ),
            # Node(
            #     name="lidar",
            #     package="rplidar_ros",
            #     executable="rplidar_composition",
            #     output="screen",
            #     parameters=[
            #         {
            #             "serial_port": "/dev/ttyUSB0",
            #             "serial_baudrate": 115200,
            #             "frame_id": "racecar/base_laser",
            #             "inverted": False,
            #             "angle_compensate": True,
            #         }
            #     ],
            #     remappings=[("/scan", "/racecar/scan")],
            # ),
            # Node(
            # package='racecar_autopilot',
            # executable='wall_estimator',
            # name='wall_estimator',
            # remappings=[('/scan','/racecar/scan')],
            # ),
        ]
    )
