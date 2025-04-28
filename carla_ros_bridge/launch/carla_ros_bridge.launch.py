import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='use_sim_time'
        ),
        launch.actions.DeclareLaunchArgument(
            name='host',
            default_value='localhost',
            description='IP of the CARLA server'
        ),
        launch.actions.DeclareLaunchArgument(
            name='port',
            default_value='2000',
            description='TCP port of the CARLA server'
        ),
        launch.actions.DeclareLaunchArgument(
            name='timeout',
            default_value='5000',
            description='Time to wait for a successful connection to the CARLA server'
        ),
        launch.actions.DeclareLaunchArgument(
            name='passive',
            default_value='False',
            description='When enabled, the ROS bridge will take a backseat and another client must tick the world (only in synchronous mode)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode',
            default_value='True',
            description='Enable/disable synchronous mode. If enabled, the ROS bridge waits until the expected data is received for all sensors'
        ),
        launch.actions.DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='False',
            description='When enabled, pauses the tick until a vehicle control is completed (only in synchronous mode)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.05',
            description='Simulation time (delta seconds) between simulation steps'
        ),
        launch.actions.DeclareLaunchArgument(
            name='start_unix_time_stamp',
            default_value='0',
            description='Start unix stamp of simulation time'
        ),
        launch.actions.DeclareLaunchArgument(
            name='town',
            #default_value='Town01',
            #default_value='Town10HD',
            default_value='/Game/aldenhoven/Maps/aldenhoven/aldenhoven',
            description='Either use an available CARLA town (eg. "Town01") or an OpenDRIVE file (ending in .xodr)'
        ),
        launch.actions.DeclareLaunchArgument(
            name='rt_factor',
            default_value='inf',
            description='Desired Realtime-Factor of the simulation'
        ),
        launch.actions.DeclareLaunchArgument(
            name='register_all_sensors',
            default_value='True',
            description='Enable/disable the registration of all sensors. If disabled, only sensors spawned by the bridge are registered'
        ),
        launch.actions.DeclareLaunchArgument(
            name='ego_vehicle_role_name',
            default_value=["hero", "ego_vehicle", "hero0", "hero1", "hero2",
                           "hero3", "hero4", "hero5", "hero6", "hero7", "hero8", "hero9"],
            description='Role names to identify ego vehicles. '
        ),
        launch.actions.DeclareLaunchArgument(
            name='publish_static_vehicles',
            default_value='True',
            description='Enable/disable object list with static vehicles'
        ),
        launch.actions.DeclareLaunchArgument(
            name='ignore_altitude',
            default_value='False',
            description='Disable altitude information'
        ),
        launch.actions.DeclareLaunchArgument(
            name='georeference_substitution',
            default_value='',
            description='Substitutes the content of the georeference xml file from the Carla OpenDrive file if not empty. Can be used to set the origin of the WorldInfo without changing the original OpenDrive file.'
        ),
        
        # etsi traffic_light parameters
        launch.actions.DeclareLaunchArgument(
            name='publish_etsi_messages',
            default_value='False',
            description=''
        ),
        
        launch.actions.DeclareLaunchArgument(
            name='waypoints_search_distance',
            default_value='1.0',
            description=''
        ),
        
        launch.actions.DeclareLaunchArgument(
            name='lane_waypoints_count',
            default_value='10',
            description=''
        ),
        
        launch.actions.DeclareLaunchArgument(
            name='taffic_light_junction_max_search_count',
            default_value='15',
            description=''
        ),
        
        launch.actions.DeclareLaunchArgument(
            name='debug_traffic_light_information',
            default_value='False',
            description=''
        ),
        
        launch.actions.DeclareLaunchArgument(
            name='integrate_junctions_without_traffic_lights',
            default_value='False',
            description=''
        ),
        
        launch.actions.DeclareLaunchArgument(
            name='traffic_light_junction_search_ignored_ids',
            default_value='[-1]', # list can not be empty
            description=''
        ),
        
        launch.actions.DeclareLaunchArgument(
            name='publisher_mapem_timer_period',
            default_value='1.0',
            description=''
        ),
        
        launch.actions.DeclareLaunchArgument(
            name='publisher_spatem_timer_period',
            default_value='0.1',
            description=''
        ),
        
        launch.actions.DeclareLaunchArgument(
            name='publisher_debug_traffic_light_information_timer_period',
            default_value='1.0',
            description=''
        ),
        
        launch_ros.actions.Node(
            package='carla_ros_bridge',
            executable='bridge',
            name='carla_ros_bridge',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
                },
                {
                    'host': launch.substitutions.LaunchConfiguration('host')
                },
                {
                    'port': launch.substitutions.LaunchConfiguration('port')
                },
                {
                    'timeout': launch.substitutions.LaunchConfiguration('timeout')
                },
                {
                    'passive': launch.substitutions.LaunchConfiguration('passive')
                },
                {
                    'synchronous_mode': launch.substitutions.LaunchConfiguration('synchronous_mode')
                },
                {
                    'synchronous_mode_wait_for_vehicle_control_command': launch.substitutions.LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command')
                },
                {
                    'fixed_delta_seconds': launch.substitutions.LaunchConfiguration('fixed_delta_seconds')
                },
                {
                    'start_unix_time_stamp': launch.substitutions.LaunchConfiguration('start_unix_time_stamp')
                },
                {
                    'town': launch.substitutions.LaunchConfiguration('town')
                },
                {
                    'rt_factor': launch.substitutions.LaunchConfiguration('rt_factor')
                },
                {
                    'register_all_sensors': launch.substitutions.LaunchConfiguration('register_all_sensors')
                },
                {
                    'ego_vehicle_role_name': launch.substitutions.LaunchConfiguration('ego_vehicle_role_name')
                },
                {
                    'publish_static_vehicles': launch.substitutions.LaunchConfiguration('publish_static_vehicles')
                },
                {
                    'ignore_altitude': launch.substitutions.LaunchConfiguration('ignore_altitude')
                },
                {
                    'georeference_substitution': launch.substitutions.LaunchConfiguration('georeference_substitution')
                },
                {
                    'publish_etsi_messages': launch.substitutions.LaunchConfiguration('publish_etsi_messages')
                },
                {
                    'waypoints_search_distance': launch.substitutions.LaunchConfiguration('waypoints_search_distance')
                },
                {
                    'lane_waypoints_count': launch.substitutions.LaunchConfiguration('lane_waypoints_count')
                },
                {
                    'taffic_light_junction_max_search_count': launch.substitutions.LaunchConfiguration('taffic_light_junction_max_search_count')
                },
                {
                    'debug_traffic_light_information': launch.substitutions.LaunchConfiguration('debug_traffic_light_information')
                },
                {
                    'integrate_junctions_without_traffic_lights': launch.substitutions.LaunchConfiguration('integrate_junctions_without_traffic_lights')
                },
                {
                    'traffic_light_junction_search_ignored_ids': launch.substitutions.LaunchConfiguration('traffic_light_junction_search_ignored_ids')
                },
                {
                    'publisher_mapem_timer_period': launch.substitutions.LaunchConfiguration('publisher_mapem_timer_period')
                },
                {
                    'publisher_debug_traffic_light_information_timer_period': launch.substitutions.LaunchConfiguration('publisher_debug_traffic_light_information_timer_period')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
