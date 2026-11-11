"""Bringup the SO-101 with ros2_control on a selectable backend (mock/gz/real)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def make_spawners(context):
    """Return the controller spawners for the chosen backend."""
    hw = LaunchConfiguration('hw').perform(context)
    motion = LaunchConfiguration('motion_controllers').perform(context)
    if motion == 'auto':
        motion = 'false' if hw == 'real' else 'true'

    # The real arm starts unconfigured, so its controllers can only be loaded and
    # configured at launch; the operator activates them after the hardware.
    extra = ['--inactive'] if hw == 'real' else []

    names = ['joint_state_broadcaster']
    if motion == 'true':
        names += ['joint_trajectory_controller', 'parallel_gripper_controller']

    return [
        Node(package='controller_manager', executable='spawner', arguments=[name] + extra)
        for name in names
    ]


def generate_launch_description():
    hw_arg = DeclareLaunchArgument(
        'hw',
        default_value='mock',
        choices=['mock', 'gz', 'real'],
        description='ros2_control backend',
    )

    motion_arg = DeclareLaunchArgument(
        'motion_controllers',
        default_value='auto',
        choices=['auto', 'true', 'false'],
        description='Spawn the trajectory and gripper controllers (auto: mock/gz yes, real no)',
    )

    gz_args_arg = DeclareLaunchArgument(
        'gz_args',
        default_value='-r empty.sdf',
        description='Arguments for gz sim (hw:=gz only); add -s for a headless server',
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='In CI testing rviz is disabled by giving false argument'
    )

    hw = LaunchConfiguration('hw')

    # Paths
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('so101_description'), 'urdf', 'so101.urdf.xacro']
    )
    controllers_file = PathJoinSubstitution(
        [FindPackageShare('so101_bringup'), 'config', 'controllers.yaml']
    )
    real_overrides_file = PathJoinSubstitution(
        [FindPackageShare('so101_bringup'), 'config', 'real_hw_overrides.yaml']
    )
    rviz_file = PathJoinSubstitution(
        [FindPackageShare('so101_bringup'), 'rviz', 'bringup.rviz']
    )

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file, ' hw:=', hw, ' gz_controllers:=', controllers_file]),
            value_type=str,
        )
    }

    is_real = EqualsSubstitution(hw, 'real')
    is_gz = EqualsSubstitution(hw, 'gz')
    # In Gazebo, time comes from the simulator (/clock), so every node must use it.
    sim_time = {'use_sim_time': ParameterValue(is_gz, value_type=bool)}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, sim_time],
        output='both',
    )

    def make_control_node(param_files, condition):
        return Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=param_files,
            remappings=[('~/robot_description', '/robot_description')],
            output='both',
            condition=condition,
        )

    # mock and real run ros2_control_node; in Gazebo the plugin hosts the controller manager.
    # hw:=real layers the overrides on top of the shared controllers.yaml.
    control_node_sim = make_control_node(
        [controllers_file], IfCondition(EqualsSubstitution(hw, 'mock')))
    control_node_real = make_control_node(
        [controllers_file, real_overrides_file], IfCondition(is_real))

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[sim_time],
    )

    # Gazebo: find the meshes (package://so101_description/... resolves under share/),
    # start the simulator, spawn the robot from /robot_description, bridge the clock.
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(get_package_share_directory('so101_description')),
        condition=IfCondition(is_gz),
    )
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items(),
        condition=IfCondition(is_gz),
    )
    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'so101'],
        output='screen',
        condition=IfCondition(is_gz),
    )
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
        condition=IfCondition(is_gz),
    )

    return LaunchDescription([
        hw_arg,
        rviz_arg,
        motion_arg,
        gz_args_arg,
        gz_resource_path,
        gz_sim,
        gz_spawn,
        clock_bridge,
        robot_state_publisher,
        control_node_sim,
        control_node_real,
        OpaqueFunction(function=make_spawners),
        rviz,
    ])
