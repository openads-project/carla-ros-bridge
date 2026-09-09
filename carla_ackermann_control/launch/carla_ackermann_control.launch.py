from pathlib import Path

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    role_name = launch.substitutions.LaunchConfiguration('role_name')
    default_ackermann_command_topic = PathJoinSubstitution([
        '/carla', role_name, 'ackermann_cmd'
    ])
    default_vehicle_control_command_topic = PathJoinSubstitution([
        '/carla', role_name, 'vehicle_control_cmd'
    ])

    remappable_topics = [
        launch.actions.DeclareLaunchArgument(
            name='ackermann_command_topic',
            default_value=default_ackermann_command_topic
        ),
        launch.actions.DeclareLaunchArgument(
            name='vehicle_control_command_topic',
            default_value=default_vehicle_control_command_topic
        )
    ]

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
            name='control_loop_rate',
            default_value='0.05'
        ),
        launch.actions.DeclareLaunchArgument(
            name='params',
            default_value=str(Path(
                get_package_share_directory('carla_ackermann_control'),
                'settings.yaml'))
        ),
        *remappable_topics,
        launch_ros.actions.Node(
            package='carla_ackermann_control',
            executable='carla_ackermann_control_node',
            name='carla_ackermann_control',
            output='screen',
            remappings=[
                (
                    default_ackermann_command_topic,
                    launch.substitutions.LaunchConfiguration(
                        'ackermann_command_topic')
                ),
                (
                    default_vehicle_control_command_topic,
                    launch.substitutions.LaunchConfiguration(
                        'vehicle_control_command_topic')
                )
            ],
            parameters=[
                launch.substitutions.LaunchConfiguration('params'),
                {
                    'role_name': role_name,
                    'control_loop_rate': launch.substitutions.LaunchConfiguration('control_loop_rate')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
