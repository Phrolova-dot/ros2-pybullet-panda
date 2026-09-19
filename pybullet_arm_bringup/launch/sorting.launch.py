"""Start a four-part visual sorting batch and its optional displays."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulation = PathJoinSubstitution(
        [FindPackageShare('pybullet_arm_bringup'), 'launch', 'simulation.launch.py'])
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('pybullet_gui', default_value='true'),
        DeclareLaunchArgument('camera_viewer', default_value='true'),
        DeclareLaunchArgument('seed', default_value='42'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(simulation),
            launch_arguments={
                'rviz': LaunchConfiguration('rviz'),
                'pybullet_gui': LaunchConfiguration('pybullet_gui'),
                'camera_viewer': LaunchConfiguration('camera_viewer'),
                'seed': LaunchConfiguration('seed'),
                'sorting_scene': 'true', 'spawn_default_box': 'false',
                'cameras': 'true', 'camera_width': '448', 'camera_height': '448',
                'camera_hz': '5.0', 'external_topic': '/sorting/annotated_image',
            }.items()),
        Node(package='pybullet_arm_control', executable='auto_sort',
             name='auto_sort', output='screen'),
    ])
