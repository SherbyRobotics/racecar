import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config_package = 'racecar_navigation'
    rviz_config_file = os.path.join(get_package_share_directory('racecar_navigation'), 'resource','rviz_navigation.rviz')

    print(rviz_config_file)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        rviz_node
    ])


