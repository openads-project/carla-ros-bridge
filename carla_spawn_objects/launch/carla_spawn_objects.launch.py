import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='use_sim_time'
        ),
        launch.actions.DeclareLaunchArgument(
            name='objects_definition_file',
            default_value='',
            description='Single object definition file (use objects_definition_files for multiple files)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='objects_definition_files',
            default_value='',
            description='Comma-separated list of object definition files to load and merge'
        ),
        launch.actions.DeclareLaunchArgument(
            name='blueprints_directory',
            default_value='',
            description='Directory containing blueprint JSON files (auto-detected if not specified)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_point_ego_vehicle',
            default_value='None'
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_sensors_only',
            default_value='False'
        ),

        launch_ros.actions.Node(
            package='carla_spawn_objects',
            executable='carla_spawn_objects',
            name='carla_spawn_objects',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
                },
                {
                    'objects_definition_file': launch.substitutions.LaunchConfiguration('objects_definition_file')
                },
                {
                    'objects_definition_files': launch.substitutions.LaunchConfiguration('objects_definition_files')
                },
                {
                    'blueprints_directory': launch.substitutions.LaunchConfiguration('blueprints_directory')
                },
                {
                    'spawn_point_ego_vehicle': launch.substitutions.LaunchConfiguration('spawn_point_ego_vehicle')
                },
                {
                    'spawn_sensors_only': launch.substitutions.LaunchConfiguration('spawn_sensors_only')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()