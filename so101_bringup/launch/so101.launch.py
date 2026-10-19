"""Bringup the SO-101 with ros2_control on a selectable backend (mock/gz/real)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hw_arg = DeclareLaunchArgument(
        'hw',
        default_value='mock',
        choices=['mock', 'gz', 'real'],
        description='ros2_control backend',
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='CI testing rviz diabled'
    )

    hw = LaunchConfiguration('hw')

    # Paths
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('so101_description'), 'urdf', 'so101.urdf.xacro']
    )
    controllers_file = PathJoinSubstitution(
        [FindPackageShare('so101_bringup'), 'config', 'controllers.yaml']
    )
    rviz_file = PathJoinSubstitution(
        [FindPackageShare('so101_bringup'), 'rviz', 'bringup.rviz']
    )

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file, ' hw:=', hw]),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='both',
    )

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_file],
        remappings=[('~/robot_description', '/robot_description')],
        output='both',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['parallel_gripper_controller']
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        hw_arg,
        rviz_arg,
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        rviz,

    ])
