import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bridge_launch_arguments = [
        ('use_sim_time', 'True'),
        ('host', 'carla-server'),
        ('port', '2000'),
        ('timeout', '5000'),
        ('passive', 'False'),
        ('publish_clock', 'True'),
        ('synchronous_mode', 'True'),
        ('synchronous_mode_wait_for_vehicle_control_command', 'False'),
        ('fixed_delta_seconds', '0.05'),
        ('start_unix_time_stamp', '0'),
        ('town', ''),
        ('rt_factor', '1.0'),
        ('register_all_sensors', 'True'),
        ('native_interface', 'True'),
        ('ego_vehicle_role_name', ['hero', 'ego_vehicle', 'hero0', 'hero1', 'hero2', 'hero3']),
        ('publish_static_vehicles', 'True'),
        ('publish_compressed_images', 'True'),
        ('ignore_altitude', 'False'),
        ('ignore_tilt', 'True'),
        ('georeference_substitution', ''),
        ('grid_convergence', 'None'),
        ('publish_etsi_messages', 'False'),
        ('publisher_mapem_timer_period', '1.0'),
        ('publisher_spatem_timer_period', '0.1'),
        ('integrate_junctions_without_traffic_lights', 'False'),
        ('traffic_light_junction_search_ignored_ids', '[-1]'),
        ('traffic_light_junction_max_search_count', '13'),
        ('waypoints_search_distance', '1.0'),
        ('lane_waypoints_count', '10'),
        ('debug_traffic_light_information', 'False'),
        ('publisher_debug_traffic_light_information_timer_period', '1.0'),
        ('log_level', 'info'),
    ]

    args = [
        *[
            DeclareLaunchArgument(
                name=name,
                default_value=default_value
            )
            for name, default_value in bridge_launch_arguments
        ],
        DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        DeclareLaunchArgument(
            name='vehicle_filter',
            default_value='vehicle.*'
        ),
        DeclareLaunchArgument(
            name='spawn_point',
            default_value='None'
        ),
    ]

    includes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_ros_bridge'), 'carla_ros_bridge.launch.py')
            ),
            launch_arguments={
                name: LaunchConfiguration(name)
                for name, _ in bridge_launch_arguments
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_spawn_objects'), 'carla_example_ego_vehicle.launch.py')
            ),
            launch_arguments={
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'timeout': LaunchConfiguration('timeout'),
                'vehicle_filter': LaunchConfiguration('vehicle_filter'),
                'role_name': LaunchConfiguration('role_name'),
                'spawn_point': LaunchConfiguration('spawn_point')
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'carla_manual_control'), 'carla_manual_control.launch.py')
            ),
            launch_arguments={
                'role_name': LaunchConfiguration('role_name')
            }.items()
        )
    ]

    return LaunchDescription([
        *args,
        *includes,
    ])


if __name__ == '__main__':
    generate_launch_description()
