from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'gui',
                default_value='true',
                description='Start RViz2 automatically with this launch file'
            ),
            # robot_state_publisher with robot_description from xacro
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[
                    {
                        'robot_description': ParameterValue(Command(
                            [
                                'xacro',
                                ' ',
                                PathSubstitution(FindPackageShare('so101_description'))
                                / 'urdf'
                                / 'so101_new_calib.urdf.xacro'
                            ]
                        ), value_type=str)
                    }
                ]
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='log',
                arguments=[
                    '-d',
                    PathSubstitution(FindPackageShare('so101_description'))
                    / 'rviz/view.rviz',
                ],
                condition=IfCondition(LaunchConfiguration('gui')),
            ),
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
            ),

        ]
    )
