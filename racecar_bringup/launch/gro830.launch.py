from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import xacro

def generate_launch_description():

    racecar_bringup = get_package_share_directory('racecar_bringup')


    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(racecar_bringup, 'launch', 'bringup.launch.py')]),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(racecar_bringup, 'launch/include', 'robot_localization.launch.py')]),
        ),
        
        Node(
            package='racecar_bringup',
            executable='cmd_vel_arbitration',
            name='cmd_vel_arbitration',
            output='screen',
            remappings=[('/cmd_vel_output', '/cmd_vel')],
        ),

        Node(
            package='joy',  
            executable='joy_node',
            name='joy',
            output='screen',
        ),

        Node(
            package='racecar_autopilot',
            executable='slash_controller',
            name='controller',
            output='screen',
            remappings=[('/ctl_ref', '/cmd_vel')],
        ),


        Node(
            package='racecar_teleop',
            executable='slash_teleop',
            name='teleop',
            output='screen',
            remappings=[('/ctl_ref', '/cmd_vel_abtr_0')],   
        ),

    ])
