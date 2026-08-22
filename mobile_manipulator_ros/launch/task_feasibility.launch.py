from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package='mobile_manipulator_validator',
                executable='task_feasibility_service_node',
                name='task_feasibility_service_node',
                output='screen',
            )
        ]
    )