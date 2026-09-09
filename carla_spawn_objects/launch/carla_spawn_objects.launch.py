import os
import sys

import launch
import launch_ros.actions
from launch_ros.parameter_descriptions import ParameterValue
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
            default_value='objects.json',
            description='Object definition file(s), comma-separated for multiple files'
        ),
        launch.actions.DeclareLaunchArgument(
            name='objects_directory',
            default_value=get_package_share_directory('carla_spawn_objects') + '/config',
            description='Base directory for object definition files (prepended to relative paths)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='blueprints_directory',
            default_value=get_package_share_directory('carla_spawn_objects') + '/config/blueprints',
            description='Base directory containing blueprint JSON files'
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
                    'objects_directory': launch.substitutions.LaunchConfiguration('objects_directory')
                },
                {
                    'blueprints_directory': launch.substitutions.LaunchConfiguration('blueprints_directory')
                },
                {
                    'spawn_point_ego_vehicle': ParameterValue(
                        launch.substitutions.LaunchConfiguration('spawn_point_ego_vehicle'),
                        value_type=str)
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
