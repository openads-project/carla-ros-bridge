# OpenADS specific changes

## General

- Support ROS 2 native interface (!67)
- Bump to CARLA 0.10.0 using Unreal Engine 5 and update Python API (!74)

## Timing Related

- Support real-time control (!7)
- Support `start_unix_time_stamp` (!24)
- Support disabling the ROS clock (!70)
- Support `use_sim_time` in all nodes (!23)

## Map Related

- Rename `frame_id` from `map` to `carla_map`
- Publish `utm_XX` to `carla_map` static transform based on georeference in OpenDRIVE data or customized by launch argument (!29, !75)
- Apply grid-convergence correction if needed (!63)

## Transform Related

- Ignore altitude (!38)
- Ignore tilt (!72)

## Spawning Related

- Add automatic spawn altitude correction for maps with elevation (!64)
- Use group and blueprint feature for `carla_spawn_objects` (!26)
- Add WGS84/global spawn point support (!56, !57)

## Sensor Related

- Compressed image support for cameras (!51)
- Optionally publish static objects (!14)
- [Ideal Object Sensor](./docs/ros_sensors.md#ideal-object-sensor) (!28)
- ETSI MAPEM/SPATEM conversion (!46)
  TODO: documented somewhere?
- Prevent similar sensor names and thus duplicated topics
- Add weather publisher (!45)

## Controlling

- Disable automated control commands while manual control is active (!32)
- Add Xbox wireless controller support (!43)
- Use `AckermannDriveStamped` instead of `AckermannDrive` (!55)

## Misc

- Disable CARLA hard version check
- Add docker-ros pipeline (!15)
