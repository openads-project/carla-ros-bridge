import os

import launch
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='objects_definition_file',
            default_value='',
            description='Single object definition file (use objects_definition_files for multiple files)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='objects_definition_files',
            default_value=get_package_share_directory(
                'carla_spawn_objects') + '/config/objects.json',
            description='Comma-separated list of object definition files to load and merge'
        ),
        launch.actions.DeclareLaunchArgument(
            name='objects_directory',
            default_value=get_package_share_directory('carla_spawn_objects') + '/config/objects',
            description='Base directory for object definition files (prepended to relative paths)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='blueprints_directory',
            default_value='',
            description='Directory containing blueprint JSON files (auto-detected if not specified)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_point_ego_vehicle',
            default_value='None'
        ),
        launch.actions.DeclareLaunchArgument(
            name='spawn_sensors_only',
            default_value='False'
        ),
        launch.actions.DeclareLaunchArgument(
            name='control_id',
            default_value='control'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_spawn_objects'), 'carla_spawn_objects.launch.py')
            ),
            launch_arguments={
                'objects_definition_file': launch.substitutions.LaunchConfiguration('objects_definition_file'),
                'objects_definition_files': launch.substitutions.LaunchConfiguration('objects_definition_files'),
                'objects_directory': launch.substitutions.LaunchConfiguration('objects_directory'),
                'blueprints_directory': launch.substitutions.LaunchConfiguration('blueprints_directory'),
                'spawn_point_ego_vehicle': launch.substitutions.LaunchConfiguration('spawn_point_ego_vehicle'),
                'spawn_sensors_only': launch.substitutions.LaunchConfiguration('spawn_sensors_only')
            }.items()
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_spawn_objects'), 'set_initial_pose.launch.py')
            ),
            launch_arguments={
                'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                'control_id': launch.substitutions.LaunchConfiguration('control_id')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()