from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    description_share = FindPackageShare('pybullet_arm_description')
    bringup_share = FindPackageShare('pybullet_arm_bringup')
    xacro_path = PathJoinSubstitution(
        [description_share, 'urdf', 'lab_arm.urdf.xacro']
    )
    rviz_config = PathJoinSubstitution(
        [description_share, 'rviz', 'lab_arm.rviz']
    )
    simulation_config = PathJoinSubstitution(
        [bringup_share, 'config', 'simulation.yaml']
    )
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path, ' use_root_inertia:=false']), value_type=str
    )

    rviz_enabled = LaunchConfiguration('rviz')
    pybullet_gui = LaunchConfiguration('pybullet_gui')
    spawn_default_box = LaunchConfiguration('spawn_default_box')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'rviz', default_value='true', description='Start RViz2'
            ),
            DeclareLaunchArgument(
                'pybullet_gui',
                default_value='false',
                description='Start the native PyBullet GUI',
            ),
            DeclareLaunchArgument(
                'spawn_default_box',
                default_value='true',
                description='Spawn the training cube',
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[
                    {
                        'robot_description': robot_description,
                        'use_sim_time': True,
                    }
                ],
            ),
            Node(
                package='pybullet_arm_sim',
                executable='simulation_node',
                name='pybullet_sim',
                output='screen',
                parameters=[
                    simulation_config,
                    {
                        'robot_description_path': xacro_path,
                        'pybullet_gui': ParameterValue(
                            pybullet_gui, value_type=bool
                        ),
                        'spawn_default_box': ParameterValue(
                            spawn_default_box, value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package='pybullet_arm_control',
                executable='trajectory_controller',
                name='trajectory_controller',
                output='screen',
                parameters=[simulation_config],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': True}],
                condition=IfCondition(rviz_enabled),
            ),
        ]
    )
