import os

import launch
import launch_ros.actions


def generate_launch_description():

    # Args that can be set from CLI
    host_launch_arg = launch.actions.DeclareLaunchArgument(
        name='host',
        default_value='localhost'
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

    scenario_launch_arg = launch.actions.DeclareLaunchArgument(
        name='scenario'
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

    available_scenarios_pub_action = launch.actions.ExecuteProcess(
        cmd=[[
            launch.substitutions.FindExecutable(name='ros2'),
            ' topic pub ',
            '/carla/available_scenarios ',
            'carla_ros_scenario_runner_types/CarlaScenarioList ',
            '"{scenarios:  [{name: ',
            launch.substitutions.LaunchConfiguration('scenario'),
            ', scenario_file: /scenarios/',
            launch.substitutions.LaunchConfiguration('scenario'),
            '.xosc}]}"'
        ]],
        shell=True,
        name='pub_scenario'
    )

    # Start scenario via service call

    execute_scenario_service_call_action = launch.actions.ExecuteProcess(
        cmd=[[
            launch.substitutions.FindExecutable(name='ros2'),
            ' service call ',
            '/scenario_runner/execute_scenario ',
            'carla_ros_scenario_runner_types/srv/ExecuteScenario '
            "\"{ 'scenario': { 'scenario_file':'/scenarios/",
            launch.substitutions.LaunchConfiguration('scenario'),
            ".xosc' } }\""
        ]],
        shell=True,
        name='launch_scenario'
    )

    # Event handler to start scenario only after the required node is started

    on_scenario_runner_node_start_event_handler = launch.actions.RegisterEventHandler(
        launch.event_handlers.OnProcessStart(
            target_action=scenario_runner_node,
            on_start=[execute_scenario_service_call_action]
        )
    )

    # Return full launch description

    return launch.LaunchDescription([
        host_launch_arg,
        port_launch_arg,
        role_name_launch_arg,
        scenario_runner_path_launch_arg,
        wait_for_ego_launch_arg,
        scenario_launch_arg,
        scenario_runner_node,
        available_scenarios_pub_action,
        on_scenario_runner_node_start_event_handler,
    ])


if __name__ == '__main__':
    generate_launch_description()