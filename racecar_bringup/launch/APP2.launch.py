from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/teleop.launch.py']),
            launch_arguments={'start_robotStatePublisher': "False",
                              'start_arduinoBridge': "True",
                              'start_arduinoSensor': "False",
                              'start_lidar': "False",
                              'start_camera': "False",
                              'start_magwick': "False",
                              'start_kalma': "False"}.items()
                                )
                                ])
