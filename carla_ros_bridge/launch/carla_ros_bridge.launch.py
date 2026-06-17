from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    bridge_parameter_names = [
        'use_sim_time',
        'host',
        'port',
        'timeout',
        'passive',
        'publish_clock',
        'synchronous_mode',
        'synchronous_mode_wait_for_vehicle_control_command',
        'fixed_delta_seconds',
        'start_unix_time_stamp',
        'town',
        'rt_factor',
        'register_all_sensors',
        'native_interface',
        'ego_vehicle_role_name',
        'publish_static_vehicles',
        'publish_compressed_images',
        'ignore_altitude',
        'ignore_tilt',
        'georeference_substitution',
        'grid_convergence',
        'publish_etsi_messages',
        'mapem_timer_period',
        'spatem_timer_period',
        'integrate_junctions_without_traffic_lights',
        'traffic_light_junction_search_ignored_ids',
        'traffic_light_junction_max_search_count',
        'waypoints_search_distance',
        'lane_waypoints_count',
    ]

    args = [
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='use_sim_time'
        ),
        DeclareLaunchArgument(
            name='host',
            default_value='carla-server',
            description='IP of the CARLA server'
        ),
        DeclareLaunchArgument(
            name='port',
            default_value='2000',
            description='TCP port of the CARLA server'
        ),
        DeclareLaunchArgument(
            name='timeout',
            default_value='5000',
            description='Time to wait for a successful connection to the CARLA server'
        ),
        DeclareLaunchArgument(
            name='passive',
            default_value='False',
            description='When enabled, the ROS bridge will take a backseat and another client must tick the world (only in synchronous mode)'
        ),
        DeclareLaunchArgument(
            name='publish_clock',
            default_value='True',
            description='When disabled, the ROS bridge will not publish clock messages (another component is expected to do so)'
        ),
        DeclareLaunchArgument(
            name='synchronous_mode',
            default_value='True',
            description='Enable/disable synchronous mode. If enabled, the ROS bridge waits until the expected data is received for all sensors'
        ),
        DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='False',
            description='When enabled, pauses the tick until a vehicle control is completed (only in synchronous mode)'
        ),
        DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.05',
            description='Simulation time (delta seconds) between simulation steps'
        ),
        DeclareLaunchArgument(
            name='start_unix_time_stamp',
            default_value='0',
            description='Start unix stamp of simulation time'
        ),
        DeclareLaunchArgument(
            name='town',
            default_value='',
            description='Either use an available CARLA town (eg. "Town01") or an OpenDRIVE file (ending in .xodr)'
        ),
        DeclareLaunchArgument(
            name='rt_factor',
            default_value='1.0',
            description='Desired Realtime-Factor of the simulation'
        ),
        DeclareLaunchArgument(
            name='register_all_sensors',
            default_value='True',
            description='Enable/disable the registration of all sensors. If disabled, only sensors spawned by the bridge are registered'
        ),
        DeclareLaunchArgument(
            name='native_interface',
            default_value='True',
            description='Enable CARLA native DDS interfaces: bridge will not create sensor publishers and control subscribers'
        ),
        DeclareLaunchArgument(
            name='ego_vehicle_role_name',
            default_value=['hero', 'ego_vehicle', 'hero0', 'hero1', 'hero2', 'hero3'],
            description='Role names to identify ego vehicles. '
        ),
        DeclareLaunchArgument(
            name='publish_static_vehicles',
            default_value='True',
            description='Enable/disable object list with static vehicles'
        ),
        DeclareLaunchArgument(
            name='publish_compressed_images',
            default_value='True',
            description='Enable/disable compressed image publishing'
        ),
        DeclareLaunchArgument(
            name='ignore_altitude',
            default_value='False',
            description='Disable altitude information'
        ),
        DeclareLaunchArgument(
            name='ignore_tilt',
            default_value='True',
            description='Disable pitch and roll information in the TF sensor output'
        ),
        DeclareLaunchArgument(
            name='georeference_substitution',
            default_value='',
            description='Substitutes the content of the georeference xml file from the CARLA OpenDRIVE file if not empty. Can be used to set the origin of the WorldInfo without changing the original OpenDRIVE file.'
        ),
        DeclareLaunchArgument(
            name='grid_convergence',
            default_value='None',
            description='Apply grid convergence when publishing the map frame transform (True/False/None for auto)'
        ),
        DeclareLaunchArgument(
            name='publish_etsi_messages',
            default_value='False',
            description='Flag if Etsi Mapem and Spatem messages should be published. Saves computation time if not needed.'
        ),
        DeclareLaunchArgument(
            name='mapem_timer_period',
            default_value='1.0',
            description='Time between publishing the Etsi Mapem message'
        ),
        DeclareLaunchArgument(
            name='spatem_timer_period',
            default_value='0.1',
            description='Time between publishing the Etsi Spatem message'
        ),
        DeclareLaunchArgument(
            name='integrate_junctions_without_traffic_lights',
            default_value='False',
            description='Flag if additionaly junctions without traffic lights should be integrated into the map. Saves computation time if not needed.'
        ),
        DeclareLaunchArgument(
            name='traffic_light_junction_search_ignored_ids',
            default_value='[-1]',  # list can not be empty
            description='In convoluted junctions, the search for traffic light junctions can output additional junctions which are not desired. Junctions with the given OpenDRIVE ids are discarded in the junction search. If empty, all junctions are searched.'
        ),
        DeclareLaunchArgument(
            name='traffic_light_junction_max_search_count',
            default_value='13',
            description='The number of waypoints to search for traffic light junctions, starting from inside each trigger box of a traffic light.'
        ),
        DeclareLaunchArgument(
            name='waypoints_search_distance',
            default_value='1.0',
            description='The search distance for waypoints in meters.'
        ),
        DeclareLaunchArgument(
            name='lane_waypoints_count',
            default_value='10',
            description='Number of waypoints included in the Etsi Mapem Egress/Ingress lanes.'
        ),
        DeclareLaunchArgument(
            name="log_level", 
            default_value="info",
            description="ROS logging level (debug, info, warn, error, fatal)"
        ),
    ]

    nodes = [
        Node(
            package='carla_ros_bridge',
            executable='bridge',
            name='carla_ros_bridge',
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            output='screen',
            emulate_tty=True,
            on_exit=Shutdown(),
            parameters=[{name: LaunchConfiguration(name)} for name in bridge_parameter_names],
        )
    ]

    return LaunchDescription([
        *args,
        SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')),
        *nodes,
    ])


if __name__ == '__main__':
    generate_launch_description()
