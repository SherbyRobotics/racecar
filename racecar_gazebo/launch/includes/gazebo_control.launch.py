import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
import launch_ros.parameter_descriptions
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    prefix = LaunchConfiguration('prefix', default='racecar')

    # Declare launch argument
    # prefix = DeclareLaunchArgument('prefix', default_value='racecar', description='Prefix for robot components')

    # load_joint_controller_config = Node(
    #     package='ros2param',
    #     executable='rosparam',
    #     arguments=['load', '/gazebo', 'gazebo_control', '-c', 'racecar_gazebo/config/gazebo_control.yaml'],
    #     output='screen'
    # )

    # load_joint_controller_config = Node(    
    #     package='ros2param',
    #     executable='param',
    #     arguments=['load', '/gazebo', 'gazebo_control', '-c', 'racecar_gazebo/config/gazebo_control.yaml'],
    #     output='screen'
    # )

    # Load controller manager node
    spawner_node = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner',
             'left_rear_wheel_velocity_controller', 'right_rear_wheel_velocity_controller',
             'left_front_wheel_velocity_controller', 'right_front_wheel_velocity_controller',
             'left_steering_hinge_position_controller', 'right_steering_hinge_position_controller',
             'joint_state_controller'],
        output='screen'
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
    )

    # spawner_node = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=[
    #         'left_rear_wheel_velocity_controller',
    #         'right_rear_wheel_velocity_controller',
    #         'left_front_wheel_velocity_controller',
    #         'right_front_wheel_velocity_controller',
    #         'left_steering_hinge_position_controller',
    #         'right_steering_hinge_position_controller',
    #         'joint_state_controller',
    #         # '-c', '/controller_manager',
    #     ],
    #     parameters=['racecar_gazebo/config/gazebo_control.yaml'],
    #     respawn=False,
    #     output='screen',
    # )


    servo_commands_node = Node(
        package='racecar_gazebo',
        executable='servo_commands',
        output='screen',
    )

    gazebo_odometry_node = Node(
        package='racecar_gazebo',
        executable='gazebo_odometry',
        arguments=[
            '-link_prefix', '{prefix}',
            '-publish_tf', 'True',
            '-frame_id', '{prefix}/base_footprint',
            '-odom_frame_id', '{prefix}/odom',
        ],
        output='screen',
    )


    cmd_vel_to_ackermann_drive_node = Node(
        package='racecar_gazebo',
        executable='cmd_vel_to_ackermann_drive',
        arguments=[
            '-frame_id', '{prefix}/odom',
        ],
        output='screen',
    )

    return LaunchDescription([
        # declare_prefix_arg,
        # load_joint_controller_config,
        # controller_manager_node,
        # robot_state_publisher_node,
        # spawner_node,
        # servo_commands_node,
        # gazebo_odometry_node,
        # cmd_vel_to_ackermann_drive_node,
    ])

if __name__ == '__main__':
    generate_launch_description()

