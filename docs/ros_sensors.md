# ROS Bridge Sensors

---

## Available Sensors

###### RGB camera

| Topic | Type |
|-------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/image` | [sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) |
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/camera_info` | [sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html) |

###### Depth camera

| Topic | Type |
|-------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/image` | [sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) |
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/camera_info` | [sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html) |

###### Semantic segmentation camera

| Topic | Type |
|-------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/image` | [sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) |
|  `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/camera_info` | [sensor_msgs/CameraInfo](http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html) |

###### DVS camera

| Topic | Type |
|-------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/events` | [sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html) |
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/image` | [sensor_msgs/Image](https://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html) |
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/camera_info` | [sensor_msgs/CameraInfo](https://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html) |

###### Lidar

| Topic | Type |
|-------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html) |

###### Semantic lidar

| Topic | Type |
|-------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html) |

###### Radar

| Topic | Type |
|-------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [sensor_msgs/PointCloud2](https://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html) |

###### IMU

| Topic | Type |
|-------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [sensor_msgs/Imu](https://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html) |

###### GNSS

| Topic | Type |
|-------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [sensor_msgs/NavSatFix](https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html) |

###### Collision Sensor

| Topic | Type |
|-------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [carla_msgs/CarlaCollisionEvent](https://github.com/carla-simulator/ros-carla-msgs/blob/master/msg/CarlaCollisionEvent.msg) |

###### Lane Invasion Sensor

| Topic | Type |
|-------|------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [carla_msgs/CarlaLaneInvasionEvent](https://github.com/carla-simulator/ros-carla-msgs/blob/master/msg/CarlaLaneInvasionEvent.msg) |

Pseudo sensors

###### TF Sensor



The tf data for the ego vehicle is published when this pseudo sensor is spawned.

Note: Sensors publish the tf data when the measurement is done. The child_frame_id corresponds with the prefix of the sensor topics.

###### Odometry Sensor

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/<PARENT ROLE NAME>/<SENSOR ROLE NAME>` | [nav_msgs/Odometry](https://docs.ros.org/en/api/nav_msgs/html/msg/Odometry.html) | Odometry of the parent actor. |

###### Speedometer Sensor

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/<PARENT ROLE NAME>/<SENSOR ROLE NAME>` | [std_msgs/Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html) | Speed of the parent actor. Units: m/s. |

###### Map Sensor

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [std_msgs/String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html) | Provides the OpenDRIVE map as a string on a latched topic. |

###### Object Sensor

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [derived_object_msgs/ObjectArray](https://docs.ros.org/en/melodic/api/derived_object_msgs/html/msg/ObjectArray.html) | Publishes all vehicles and walker. If attached to a parent, the parent is not contained. |

###### Ideal Object Sensor

The sensor detects objects/targets (vehicles and walkers) in a specified field of view and range. The visibility of an object is checked using the corners of the object's `bounding_box`.

The visibility of a target is evaluated using 4 filters:
1. Filter: Rough filtering of targets based on the distance between the sensor and the target's center. The target is kept for the following filters if the center is within `range + target_center_range_margin`.
2. Filter: Distance measurement from the sensor to all vertices of the target's `bounding_box`.
3. Filter: Filtering vertices by checking their location in the sensor field of view using azimuth and elevation from the sensor to the vertex in the sensor coordinate frame.
4. Filter: If `enable_occlusion_filter` is set, filtering vertices by sending a ray from the sensor location to the vertex location to check if vertices are occluded by other objects.

FILTER 4 is deactivated by default. Set `enable_occlusion_filter` to `true` to activate it. When FILTER 4 is active, `hit_point_blanking_radius` defines a radius around the sensor in which ray hit points are ignored. This can be used to ignore hits on the ego vehicle or sensor mount close to the sensor.

The following parameters can be set. Distances and field-of-view angles are evaluated relative to the sensor coordinate frame, with distances measured from the sensor origin:

| Parameter | Unit | Type | <div style="text-align: center">Default Value</div> | <div style="text-align: center">Range of Applicability</div> | Description |
|-----------|------|------|---------------|------------------------|-------------|
| `range` | Meters | `float` | <div style="text-align: right">100</div> | <div style="text-align: center">[0 , ∞]</div> | Range of the sensor, measured from the sensor origin |
| `left_fov` | Degrees | `float` | <div style="text-align: right">-180</div> | <div style="text-align: center">[-180 , 0]</div> | Sensor's left field of view in the sensor coordinate frame |
| `right_fov` | Degrees | `float` | <div style="text-align: right">180</div> | <div style="text-align: center">[0 , 180]</div> | Sensor's right field of view in the sensor coordinate frame |
| `upper_fov` | Degrees | `float` | <div style="text-align: right">90</div> | <div style="text-align: center">[0 , 90]</div> | Sensor's upper field of view in the sensor coordinate frame |
| `lower_fov` | Degrees | `float` | <div style="text-align: right">-90</div> | <div style="text-align: center">[-90 , 0]</div> | Sensor's lower field of view in the sensor coordinate frame |
| `min_corner_amount` | <div style="text-align: center">-</div> | `int` | <div style="text-align: right">1</div> | <div style="text-align: center">[1 , 8]</div> | Number of vertices required for a precise detection of the target. Must be a whole number. |
| `target_center_range_margin` | Meters | `float` | <div style="text-align: right">10</div> | <div style="text-align: center">[0 , ∞]</div> | Additional margin for the rough target-center range check in FILTER 1. This prevents large targets from being discarded only because their center is just outside `range` while parts of their `bounding_box` may still be inside the sensor range. |
| `enable_occlusion_filter` | <div style="text-align: center">-</div> | `bool` | <div style="text-align: right">false</div> | <div style="text-align: center">true / false</div> | Enables FILTER 4. If disabled, object visibility is evaluated only by range, field of view, and the required number of bounding-box vertices. |
| `hit_point_blanking_radius` | Meters | `float` | <div style="text-align: right">0</div> | <div style="text-align: center">[0 , ∞]</div> | Radius around the sensor in which ray hit points are ignored while FILTER 4 is active. |

The sensor publishes on the following topic:

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [derived_object_msgs/ObjectArray](https://docs.ros.org/en/melodic/api/derived_object_msgs/html/msg/ObjectArray.html) | Publishes all visible vehicles and walker. If attached to a parent, the parent is not contained. If attached directly to the world, `[<PARENT ROLE NAME>]` does not exist. |

###### Marker Sensor

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [visualization_msgs/Marker](https://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html) | Visualization of vehicles and walkers |

###### Traffic Lights Sensor

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/status` | [carla_msgs/CarlaTrafficLightStatusList](https://github.com/carla-simulator/ros-carla-msgs/blob/master/msg/CarlaTrafficLightStatusList.msg) | List of all traffic lights with their status. |
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/info` | [carla_msgs/CarlaTrafficLightInfoList](https://github.com/carla-simulator/ros-carla-msgs/blob/master/msg/CarlaTrafficLightInfoList.msg) | Static information for all traffic lights (e.g. position). |

###### Actor List Sensor

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>` | [carla_msgs/CarlaActorList](https://github.com/carla-simulator/ros-carla-msgs/blob/master/msg/CarlaActorList.msg) | List of all CARLA actors. |

###### Actor Control Sensor

This pseudo-sensor allows to control the position and velocity of the actor it is attached to (e.g. an ego_vehicle) by publishing pose and velocity within Pose and Twist datatypes. CAUTION: This control method does not respect the vehicle constraints. It allows movements impossible in the real world, like flying or rotating. Currently this sensor applies the complete linear vector, but only the yaw from angular vector.

| Topic | Type | Description |
|-------|------|-------------|
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/set_transform` | [geometry_msgs/Pose](https://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html) | Transform to apply to the sensor's parent. |
| `/carla/[<PARENT ROLE NAME>]/<SENSOR ROLE NAME>/set_target_velocity` | [geometry_msgs/Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html) | Velocity (angular and linear) to apply to the sensor's parent. |
