from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    description_share = FindPackageShare('pybullet_arm_description')
    xacro_path = PathJoinSubstitution(
        [description_share, 'urdf', 'lab_arm.urdf.xacro']
    )
    rviz_config = PathJoinSubstitution(
        [description_share, 'rviz', 'lab_arm.rviz']
    )
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path, ' use_root_inertia:=false']), value_type=str
    )

    return LaunchDescription(
        [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description}],
            ),
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                output='screen',
                parameters=[{'robot_description': robot_description}],
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
            ),
        ]
    )
