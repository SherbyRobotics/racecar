from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    racecar_bringup = get_package_share_directory('racecar_bringup')
    
    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(racecar_bringup, 'launch', 'bringup.launch.py')]),
                        launch_arguments={'start_robotStatePublisher': "False",
                              'start_arduinoBridge': "True",
                              'start_arduinoSensor': "True",
                              'start_lidar': "False",
                              'start_camera': "False",
                              'start_magwick': "False"}.items()
        ),
