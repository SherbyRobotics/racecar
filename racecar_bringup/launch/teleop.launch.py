from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import datetime

def generate_launch_description():
    return LaunchDescription([
        
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='pb2ros2',
                    executable='arduino_agent',
                    name='arduino',
                    output='screen',
                ),
            ],
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
