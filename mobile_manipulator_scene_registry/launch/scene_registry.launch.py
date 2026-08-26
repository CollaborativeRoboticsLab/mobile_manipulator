from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare('mobile_manipulator_scene_registry'), 'config', 'scenes.yaml']
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('scenes_file', default_value=default_config),
            DeclareLaunchArgument(
                'selector_plugin',
                default_value='mobile_manipulator_scene_selector::ExactNameThenTagsSelector',
            ),
            DeclareLaunchArgument('activate_scene_action_name', default_value='activate_scene'),
            DeclareLaunchArgument('get_active_scene_service_name', default_value='get_active_scene'),
            Node(
                package='mobile_manipulator_scene_registry',
                executable='scene_registry_node',
                name='scene_registry_node',
                output='screen',
                parameters=[
                    {
                        'scenes_file': LaunchConfiguration('scenes_file'),
                        'selector_plugin': LaunchConfiguration('selector_plugin'),
                        'activate_scene_action_name': LaunchConfiguration('activate_scene_action_name'),
                        'get_active_scene_service_name': LaunchConfiguration('get_active_scene_service_name'),
                    }
                ],
            )
        ]
    )