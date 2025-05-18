import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    racecar_bringup = get_package_share_directory('racecar_bringup')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(racecar_bringup, 'launch', 'bringup.launch.py')]),
        ),

        Node(
            package='racecar_teleop',
            executable='slash_teleop',
            name='teleop',
            output='screen',
        ),

        Node(
            package='racecar_autopilot',
            executable='slash_controller',
            name='controller',
            output='screen',
        ),

        TimerAction(
            period=5.0,  # Adjust the delay duration as needed (in seconds)
            actions=[
                Node(
                    package='joy',
                    executable='joy_node',
                    name='joy',
                    parameters=[{'deadzone': 0.05}],
                    arguments=['dev', '/dev/input/js0'],
                    output='screen',
                ),
            ]
        ),
    ])
