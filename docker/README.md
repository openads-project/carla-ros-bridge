# ROS-Bridge

This repository aims to provide standalone images of the [CARLA ROS Bridge](https://gitlab.ika.rwth-aachen.de/fb-fi/simulation/carla/ros-bridge) fork by **ika** for usage in CI pipelines, cluster deployments etc. by utilizing [docker-ros](https://gitlab.ika.rwth-aachen.de/fb-fi/ops/docker-ros).

- [ROS-Bridge](#ros-bridge)
  - [Nodes](#nodes)
    - [carla\_ros\_bridge/bridge.py](#carla_ros_bridgebridgepy)
      - [Subscribed Topics](#subscribed-topics)
      - [Published Topics](#published-topics)
      - [Services](#services)
  - [Usage of docker-ros Images](#usage-of-docker-ros-images)
    - [Available Images](#available-images)
    - [Default Command](#default-command)
    - [Launch Files](#launch-files)
  - [Building Locally](#building-locally)
    - [Requirements](#requirements)
    - [Steps](#steps)
  - [Official Documentation](#official-documentation)


## Nodes

| Package | Node | Description |
| --- | --- | --- |
| `carla_ros_bridge` | `bridge.py` | Enables two-way communication between ROS and CARLA via ROS topics and commands applied to the CARLA server |

### carla_ros_bridge/bridge.py

This node, besides other components, comes from the ika fork of the [ros-bridge](https://gitlab.ika.rwth-aachen.de/fb-fi/simulation/carla/ros-bridge).

Also see the [official documentation](https://carla.readthedocs.io/projects/ros-bridge/en/latest/)

#### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/carla/debug_marker` | `visualization_msgs/MarkerArray` | Draws markers in the CARLA world. |
| `/carla/weather_control` | `carla_msgs/CarlaWeatherParameters` | Set the CARLA weather parameters |
| `/clock` | `rosgraph_msgs/Clock` | Publishes simulated time in ROS. |

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `/carla/status` | `carla_msgs/CarlaStatus` | Read the current status of CARLA |
| `/carla/world_info` | `carla_msgs/CarlaWorldInfo` | Information about the current CARLA map. |
| `/clock` | `rosgraph_msgs/Clock` | Publishes simulated time in ROS. |
| `/rosout` | `rosgraph_msgs/Log` | ROS logging. |

#### Services

| Service | Type | Description |
| --- | --- | --- |
| `/carla/destroy_object` | `carla_msgs/DestroyObject.srv` | Destroys an object |
| `/carla/get_blueprints` | `carla_msgs/GetBlueprints.srv` | Gets blueprints |
| `/carla/spawn_object` | `carla_msgs/SpawnObject.srv` | Spawn an object |


## Usage of docker-ros Images

### Available Images

| Tag | Description |
| --- | --- |
| `latest-dev_main_ci-amd64 ` | latest dev version |
| `latest_main_ci-amd64 ` | latest run version |

### Default Command

```bash
bash -ic "roslaunch carla_ros_bridge carla_ros_bridge.launch"
```
**Note:** The `-i` flag is used to invoke an interactive shell so any additions/changes to the `.bashrc` are effective when running the command.

### Launch Files

| Package | File | Path | Description |
| --- | --- | --- | --- |
| `carla_ros_bridge` | `carla_ros_bridge(launch/py)` | `/docker-ros/ws/install/share/carla_ros_bridge` | Creates ROS bridge node and connects it to the CARLA server |


## Building Locally

### Requirements

For a build outside of the GitLab CI you will need to provide a few parts of the CARLA PythonAPI that are usually downloaded and integrated automatically. These parts should be placed into `docker/files/carla` (create, if not present)

The above mentioned `carla` directory should look like this:

```bash
carla
├── agents
│   ├── navigation
│   │   ├── ...
│   ├── tools
│   │   ├── ...
│   ├── __init__.py
└── dist
    ├── carla-0.9.15-cp310-cp310m-linux_x86_64.whl 
    ├── carla-0.9.15-py3.10-linux-x86_64.egg 
```

### Steps

1) Clone repository
2) Meet requirements mentioned above
3) Navigate to `docker` directory of cloned repository
4) Make changes to `docker-compose.yml` if necessary
5) Run `docker compose build dev run` and wait for completion
6) Start image (e.g. `docker run gitlab.ika.rwth-aachen.de:5050/fb-fi/simulation/carla/ros-bridge:latest`)

## Official Documentation

- [ROS Bridge](https://carla.readthedocs.io/projects/ros-bridge/en/latest/)
