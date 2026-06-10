import os
import glob
import xml.etree.ElementTree as ET

import launch
import launch_ros.actions


def _normalized_map_reference(map_value):
    value = map_value.strip().replace('\\', '/')
    if not value:
        return ''

    filename = value.rsplit('/', 1)[-1]
    if filename.lower().endswith('.xodr'):
        value = filename[:-len('.xodr')]

    return value.lower()


def _xosc_map_reference(xosc_file):
    try:
        root = ET.parse(xosc_file).getroot()
    except (OSError, ET.ParseError):
        return ''

    for element in root.iter():
        if element.tag.rsplit('}', 1)[-1] == 'LogicFile':
            return _normalized_map_reference(element.attrib.get('filepath', ''))

    return ''


def _create_available_scenarios_pub_action(context, *args, **kwargs):
    scenarios_dir = launch.substitutions.LaunchConfiguration(
        'scenarios_dir').perform(context)
    selected_map = _normalized_map_reference(os.environ.get('SCENARIO_MAP', ''))
    scenario_files = []

    if os.path.exists(scenarios_dir):
        xosc_files = glob.glob(os.path.join(scenarios_dir, '**', '*.xosc'), recursive=True)
        for xosc_file in sorted(xosc_files):
            # Skip files in catalog subdirectories
            if '/catalog' in xosc_file:
                continue
            if selected_map and _xosc_map_reference(xosc_file) != selected_map:
                continue
            # Get relative path and scenario name
            scenario_path = os.path.relpath(xosc_file, scenarios_dir)
            scenario_name = os.path.splitext(scenario_path)[0]
            scenario_path = os.path.join(scenarios_dir, scenario_path)
            scenario_files.append(f'{{name: {scenario_name}, scenario_file: {scenario_path}}}')
    scenarios_list = ''
    if scenario_files:
        scenarios_list += ','.join(scenario_files)

    return [
        launch.actions.ExecuteProcess(
            cmd=[[
                launch.substitutions.FindExecutable(name='ros2'),
                ' topic pub ',
                '/carla/available_scenarios ',
                'carla_ros_scenario_runner_types/CarlaScenarioList ',
                '"{scenarios:  [',
                scenarios_list,
                ']}"'
            ]],
            shell=True,
            name='pub_scenario'
        )
    ]


def generate_launch_description():

    # Args that can be set from CLI
    use_sim_time_launch_arg = launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True'
    )

    host_launch_arg = launch.actions.DeclareLaunchArgument(
        name='host',
        default_value='carla-server'
    )

    port_launch_arg = launch.actions.DeclareLaunchArgument(
        name='port',
        default_value='2000'
    )

    role_name_launch_arg = launch.actions.DeclareLaunchArgument(
        name='role_name',
        default_value='ego_vehicle'
    )

    scenario_runner_path_launch_arg = launch.actions.DeclareLaunchArgument(
        name='scenario_runner_path'
    )

    wait_for_ego_launch_arg = launch.actions.DeclareLaunchArgument(
        name='wait_for_ego',
        default_value='True'
    )

    scenarios_dir_launch_arg = launch.actions.DeclareLaunchArgument(
        name='scenarios_dir',
        default_value='/scenarios'
    )

    # Start scenario runner ROS node

    scenario_runner_node = launch_ros.actions.Node(
        package='carla_ros_scenario_runner',
        executable='carla_ros_scenario_runner',
        name='carla_ros_scenario_runner',
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
                'role_name': launch.substitutions.LaunchConfiguration('role_name')
            },
            {
                'scenario_runner_path': launch.substitutions.LaunchConfiguration('scenario_runner_path')
            },
            {
                'wait_for_ego': launch.substitutions.LaunchConfiguration('wait_for_ego')
            }
        ]
    )

    # Publish available scenario

    available_scenarios_pub_action = launch.actions.OpaqueFunction(
        function=_create_available_scenarios_pub_action
    )

    # Start scenario via service call

    execute_scenario_service_call_action = launch.actions.ExecuteProcess(
        cmd=[[
            launch.substitutions.FindExecutable(name='ros2'),
            ' service call ',
            '/scenario_runner/execute_scenario ',
            'carla_ros_scenario_runner_types/srv/ExecuteScenario '
            "\"{ 'scenario': { 'scenario_file':'",
            launch.substitutions.LaunchConfiguration('scenarios_dir'),
            "/",
            launch.substitutions.LaunchConfiguration('scenario'),
            ".xosc' } }\""
        ]],
        shell=True,
        name='launch_scenario'
    )

    # Return full launch description

    return launch.LaunchDescription([
        use_sim_time_launch_arg,
        host_launch_arg,
        port_launch_arg,
        role_name_launch_arg,
        scenario_runner_path_launch_arg,
        wait_for_ego_launch_arg,
        scenarios_dir_launch_arg,
        scenario_runner_node,
        available_scenarios_pub_action
    ])


if __name__ == '__main__':
    generate_launch_description()