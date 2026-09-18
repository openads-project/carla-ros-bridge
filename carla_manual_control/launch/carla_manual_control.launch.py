import os
import sys

import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='log_level',
            default_value='info',
            description='ROS logging level (debug, info, warn, error, fatal)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Use simulation clock if True'
        ),
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
            name='wireless_controller',
            default_value='False'
        ),
        launch.actions.DeclareLaunchArgument(
            name='window_width',
            default_value='800'
        ),
        launch.actions.DeclareLaunchArgument(
            name='window_height',
            default_value='600'
        ),
        launch_ros.actions.Node(
            package='carla_manual_control',
            executable='carla_manual_control',
            name=['carla_manual_control_', launch.substitutions.LaunchConfiguration('role_name')],
            arguments=[
                '--ros-args', '--log-level',
                launch.substitutions.LaunchConfiguration('log_level')
            ],
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'use_sim_time':
                        launch.substitutions.LaunchConfiguration(
                            'use_sim_time'),
                    'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                    'wireless_controller': launch.substitutions.LaunchConfiguration('wireless_controller'),
                    'window_width': launch.substitutions.LaunchConfiguration('window_width'),
                    'window_height': launch.substitutions.LaunchConfiguration('window_height')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
