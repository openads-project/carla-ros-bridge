# IKA specific changes

## Not relevant changes for publication
- rename frame_id to carla_map (@gkueppers)
- control rt_factor (!7)
- extend standard launch file of carla_spawn_objects package (!16)
- prevent similar role names for sensors (@hermens)
- add GitLab CI Pipeline (!9)

## Interesting for an official merge request
- publish_static_vehicles in object lists (!14)
- publish destination point in ros_vehicle_control.py (@gkueppers)
- use_sim_time for ROS nodes (!23)
- control start_unix_time_stamp (!24)
- add blueprint and groups feature for sensor definitions (!26)
- add ideal object sensor (!28)
- publish utm -> carla_map tf based on OpenDrive header (!29)
- additional parameter ignore_altitude to disable height information (!38)
