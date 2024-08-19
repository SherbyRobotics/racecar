from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():


    return LaunchDescription([

        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_node',
            output='screen', 
            parameters=[
                {'use_mag':True},
                {'world_frame':'enu'},
                {'publish_tf':False}
            ]
        ),
        

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("racecar_bringup"), 'launch', 'include', 'racecar_ekf_params.yaml')],
           ),
    ])
