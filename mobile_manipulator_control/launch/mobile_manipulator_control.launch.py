from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    scene_registry_share = get_package_share_directory('mobile_manipulator_scene_registry')
    default_config = PathJoinSubstitution(
        [FindPackageShare('mobile_manipulator_control'), 'config', 'mobile_manipulator_control.yaml']
    )
    default_scenes = PathJoinSubstitution(
        [FindPackageShare('mobile_manipulator_scene_registry'), 'config', 'scenes.yaml']
    )
    scene_registry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(scene_registry_share) / 'launch' / 'scene_registry.launch.py')
        ),
        launch_arguments={
            'scenes_file': LaunchConfiguration('scenes_file'),
            'activate_scene_action_name': LaunchConfiguration('activate_scene_action_name'),
            'get_active_scene_service_name': LaunchConfiguration('get_active_scene_service_name'),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('config_file', default_value=default_config),
            DeclareLaunchArgument('scenes_file', default_value=default_scenes),
            DeclareLaunchArgument('activate_scene_action_name', default_value='activate_scene'),
            DeclareLaunchArgument('get_active_scene_service_name', default_value='get_active_scene'),
            scene_registry_launch,
            Node(
                package='mobile_manipulator_control',
                executable='mobile_manipulator_control_node',
                name='mobile_manipulator_control_node',
                output='screen',
                parameters=[LaunchConfiguration('config_file')],
            )
        ]
    )