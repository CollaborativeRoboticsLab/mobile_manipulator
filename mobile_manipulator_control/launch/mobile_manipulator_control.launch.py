from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare('mobile_manipulator_control'), 'config', 'mobile_manipulator_control.yaml']
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument('config_file', default_value=default_config),
            Node(
                package='mobile_manipulator_control',
                executable='mobile_manipulator_control_node',
                name='mobile_manipulator_control_node',
                output='screen',
                parameters=[LaunchConfiguration('config_file')],
            )
        ]
    )