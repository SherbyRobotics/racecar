from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/teleop.launch.py']),
            launch_arguments={'start_camera': "False"}.items()
        ),

        Node(
            package='racecar_autopilot',
            executable='wall_estimator',
            name='wall_estimator',
        ),
        
    ])
