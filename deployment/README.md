# CARLA ROS bridge deployments

This directory contains one deployment per runnable component provided by this
repository. The files follow the output structure and conventions of the
`openads-dev-environment` Compose and Helm generators, but are maintained
manually because this repository packages several ROS packages in one image.

| Component | Compose file | Helm chart |
| --- | --- | --- |
| CARLA ROS bridge | `compose/docker-compose.carla_ros_bridge.yml` | `helm/carla_ros_bridge/` |
| Object and sensor spawning | `compose/docker-compose.carla_spawn_objects.yml` | `helm/carla_spawn_objects/` |
| Ackermann control | `compose/docker-compose.carla_ackermann_control.yml` | `helm/carla_ackermann_control/` |
| Manual control | `compose/docker-compose.carla_manual_control.yml` | `helm/carla_manual_control/` |

All deployments use the same `carla-ros-bridge` image. They contain only
repository-owned launch files and their launch arguments. Simulation-specific
composition stays in the consuming stack. This includes profiles, startup
dependencies, middleware configuration, networks, custom launch files, map and
configuration mounts, X11 access, devices, privileges, and topic remappings.

## Docker Compose

Compose files can be used separately or combined:

```sh
docker compose \
  -f deployment/compose/docker-compose.carla_ros_bridge.yml \
  -f deployment/compose/docker-compose.carla_spawn_objects.yml \
  -f deployment/compose/docker-compose.carla_ackermann_control.yml \
  up -d
```

The generated-style environment variable names map directly to launch
arguments. Relevant examples are `HOST`, `PORT`, `TOWN`,
`OBJECTS_DEFINITION_FILE`, `OBJECTS_DIRECTORY`, `BLUEPRINTS_DIRECTORY`,
`SPAWN_POINT_EGO_VEHICLE`, `ROLE_NAME`, `CONTROL_LOOP_RATE`, `PARAMS`,
`ACKERMANN_COMMAND_TOPIC`, and `VEHICLE_CONTROL_COMMAND_TOPIC`.

A consuming stack can extend these services and replace environment, command,
volumes, profiles, and dependencies. For example, OpenADSim mounts its custom
OpenDRIVE map and launch files, maps `SPAWN_POINT` and `SENSORS` to the spawn
launch arguments, and supplies the Ackermann parameter file there. None of
those stack-owned files are duplicated here.

## Helm

Each component is an independent chart using the shared `openadservice` chart:

```sh
helm dependency build deployment/helm/carla_ros_bridge
helm upgrade --install carla-ros-bridge deployment/helm/carla_ros_bridge \
  --namespace simulation --create-namespace
```

Install the other charts in the same way when needed. Override values through
the usual `openadservice` interface, such as `openadservice.env`,
`openadservice.args`, `openadservice.volumes`, `openadservice.image`, and the
remaining scheduling and runtime options supported by the shared chart.

Manual control needs display access and may need controller devices or elevated
permissions. Those settings depend on the target cluster and therefore belong
in the stack's Helm values rather than this repository's defaults.

The root `helm-oci` workflow discovers and publishes the individual chart
directories. When launch arguments or image versions change, update the
corresponding Compose and Helm files together.
