from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    bringup_share = FindPackageShare('pybullet_arm_bringup')
    simulation_launch = PathJoinSubstitution(
        [bringup_share, 'launch', 'simulation.launch.py']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('rviz', default_value='true'),
            DeclareLaunchArgument('pybullet_gui', default_value='false'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation_launch),
                launch_arguments={
                    'rviz': LaunchConfiguration('rviz'),
                    'pybullet_gui': LaunchConfiguration('pybullet_gui'),
                }.items(),
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package='pybullet_arm_control',
                        executable='demo_sequence',
                        name='demo_sequence',
                        output='screen',
                    )
                ],
            ),
        ]
    )
