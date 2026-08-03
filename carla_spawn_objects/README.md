# CARLA Spawn Objects

Find the official documentation about the CARLA Spawn Objects package [__here__](https://carla.readthedocs.io/projects/ros-bridge/en/latest/carla_spawn_objects/). The following README contains information about OpenADS specific changes regarding the `carla_spawn_objects` package.

---

## Object Configuration

Vehicles and sensors can be defined using a configuration file (see [objects.json](./config/objects.json)). This is used to spawn a specific vehicle and its sensor equipment in a reproducible setup. Besides the two object types `vehicle` and `sensor`, two _placeholder_ entities are added, namely the `group` and `blueprint` type. All four entities are further described in the following.

### Vehicle

A vehicle (or walker) directly represents a CARLA actor which can be moved dynamically within the world. There are some important remarks for the configuration of vehicles:

- a vehicle has to be configured at top-level within the `objects` section
- the type has to start with `vehicle.`
- a vehicle can contain multiple `children` of type sensor, group, or blueprint

Thus, a <vehicle_configuration> can be configured as:
```
{
  "id": "<vehicle_name>",
  "type": "vehicle.<vehicle_model>",
  "children": [
    <sensor_configuration>,
    <sensor_configuration>,
    ...
    <group_configuration>,
    <group_configuration>,
    ...
    <blueprint_configuration>,
    <blueprint_configuration>,
    ...
  ],
  ...
}
```

### Sensor

A sensor represents a new CARLA sensor. Sensors can act globally or can be attached to actors (vehicles and walkers). The sensor configuration is consistent with the original `carla_spawn_objects` package. However, there are some important remarks for the configuration of sensors:

- the type has to start with `sensor.`
- a sensor does not have any children
- a sensor can contain `attached_objects`, which are spawned after the sensor and attached directly to the sensor actor
- `attached_objects` may only contain sensor configurations with a type starting with `sensor.`

Thus, a <sensor_configuration> can be configured as:
```
{
  "id": "<sensor_name>",
  "type": "sensor.<sensor_type>",
  "attached_objects": [
    <sensor_configuration>,
    ...
  ],
  ...
}
```

### Groups

A group does not represent any CARLA entity, but acts as a placeholder which can aggregate different other entities within a common coordinate system. It can be used to mimic a vendor-specific lidar sensor by combining multiple sensors with fixed relative transformations. Thus, children of a group are recursively configured based on the groups transformation. However, there are some important remarks for the configuration of groups:

- the type has to start with `group.`
- a group can contain multiple `children` of type sensor, group, or blueprint
- a group can also represent an _optional_ physical static object, which is then spawed within CARLA

Thus, a <group_configuration> can be configured as:
```
{
  "id": "<group_name>",
  "type": "group",
  "physical_object": "static.<model>",
  "children": [
    <sensor_configuration>,
    <sensor_configuration>,
    ...
    <group_configuration>,
    <group_configuration>,
    ...
    <blueprint_configuration>,
    <blueprint_configuration>,
    ...
  ],
}
```

### Blueprints

A blueprint does not represent any CARLA entity, but acts as a placeholder. A blueprint can be defined in a special blueprint section in an object definition file or in a JSON file below the configured `blueprints_directory`. A blueprint can represent a vehicle, sensor, group, but not another blueprint.
```
{
  "blueprints":
  [
    <vehicle_configuration>
    <vehicle_configuration>
    ...
    <sensor_configuration>,
    <sensor_configuration>,
    ...
    <group_configuration>,
    <group_configuration>,
    ...
  ]
}
```

The node loads all JSON files below `blueprints_directory` recursively and merges their `blueprints` sections before loading the configured object definition files. `objects_definition_file` can contain a single file or a comma-separated list of files. Relative object definition paths are resolved against `objects_directory`.

Blueprint ids must be unique. If a duplicate id is encountered, the first definition wins and later definitions with the same id are skipped with a warning. Auto-loaded blueprints from `blueprints_directory` are loaded before inline blueprints from `objects_definition_file`, so inline definitions cannot override an auto-loaded blueprint with the same id.

Those defined blueprints can then be used within the `objects` section in different configurations. Either on top-level, as group child, or vehicle child. A specific blueprint can be selected by using the type `blueprint.<blueprint_id>`.

Thus, the prefix `blueprint.` is required to configure a <blueprint_configuration>. A blueprint usage can override the resulting object's `id` and, optionally, its `spawn_point`; other fields come from the blueprint definition.
```
{
  "id": "<blueprint_name>",
  "type": "blueprint.<blueprint_id>",
}
```

### Spawn Points

A `spawn_point` places an object within the world. It can be given either in CARLA coordinates or in WGS84 coordinates:
```
"spawn_point": {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
"spawn_point": {"lat": 50.779692, "lon": 6.050775, "alt": 255.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
```

WGS84 spawn points are projected into the UTM zone of the given longitude and then transformed into the `carla_map` frame, which requires the `utm_<zone>` -> `carla_map` transform published by the bridge.

#### Altitude Above Ground

Instead of an absolute altitude, the altitude can be given relative to the terrain by using `z_above_ground` resp. `alt_above_ground`:
```
"spawn_point": {"lat": 50.779692, "lon": 6.050775, "alt_above_ground": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
```

The bridge then determines the ground altitude below the object and spawns it at `ground + <value>`. This is useful for e.g. roadside units, whose mounting height is known while the exact terrain altitude at their position is not. Notes:

- the correction is applied within CARLA only; in ROS the object keeps its ground-relative altitude, which is also what its transform reports
- the property is inherited by all `children`, so a group only has to declare it at its root
- it only takes effect for objects that are not attached to another actor, since an attached object is always placed relative to its parent
- `alt`/`z` and `alt_above_ground`/`z_above_ground` are mutually exclusive; if both are given, the ground-relative one is used
- a plain `alt`/`z` stays an absolute map altitude and is never corrected against the terrain, so an object placed that way keeps its altitude even where the ground is higher or lower
- groups are not required; a single sensor defined at top-level can use a ground-relative spawn point just as well
- the bridge parameter [`ignore_altitude`](../docs/run_ros.md) flattens the transform of an *actor* to `z = 0`, but never the transform of a *sensor*, which keeps its mounting height relative to its parent. An unattached sensor has no such parent, so only a ground-relative spawn point places it consistently with that flattened ground plane; with an absolute altitude it ends up as far above the flattened actors as the terrain is high

### Transforms

The node broadcasts a static transform for every group it resolves, from the parent group (or `carla_map`) to the group itself, which makes the group hierarchy visible in TF.

The transform of a sensor is normally broadcast by the CARLA server through its native ROS 2 interface, relative to the actor the sensor is attached to. A sensor that is not attached to an actor has no such parent, so the server would broadcast it against `carla_map` at its absolute pose within the world, which does not reflect the group hierarchy.

This node therefore takes the frame over for every sensor that is spawned without a parent actor and is no pseudo sensor. It spawns them with the CARLA attribute `no_transform` and broadcasts the static transform from the enclosing group (or `carla_map`) to the sensor itself. Only the transform changes its source, the sensor's data topics are unaffected. Pseudo sensors are excluded, as they are no CARLA actors and are handled by the bridge.

This applies to those sensors unconditionally. It does not depend on `ignore_altitude`, on the use of groups, or on the kind of spawn point: a single sensor defined at top-level is taken over just as one nested in a group.

The decision can be overridden per sensor by setting `no_transform` explicitly, which then determines both the CARLA attribute and who broadcasts the transform:
```
{
  "id": "<sensor_name>",
  "type": "sensor.<sensor_type>",
  "no_transform": false
}
```

- `false` on an unattached sensor keeps the server-side transform, so the sensor is placed against `carla_map` at its absolute pose
- `true` on an attached sensor moves its transform to this node, which broadcasts it relative to the enclosing group

This requires a CARLA server that supports the `no_transform` attribute. Against an older server the bridge logs a warning, ignores the attribute, and the sensor keeps its server-side transform.

## Testcases

Vehicles, groups and blueprints can contain other entities through `children`. Therefore, 16 different direct nesting configurations can be posed, where only some of them are valid with respect to the current implementation. The following table gives an overview of all supported direct `children` cases. The row is the parent object and the column is the child object.

| parent \ child | **vehicle** | **sensor** | **group** | **blueprint** |
|----------------|-------------|------------|-----------|---------------|
| **vehicle**    | _invalid_   | valid      | valid     | valid         |
| **sensor**     | _invalid_   | _invalid_  | _invalid_ | _invalid_     |
| **group**      | _invalid_   | valid      | valid     | valid         |
| **blueprint**  | valid       | valid      | valid     | _invalid_     |

`attached_objects` on sensors are a separate mechanism and are not covered by the `children` matrix. They are valid only for sensor configurations. Actor pseudo-objects, vehicles, groups and blueprints are not valid `attached_objects`.

In addition all valid configurations are somehow contained in the example file [test_sensors_supported.json](./config/test_sensors_supported.json), whereas all invalid configurations are tested in [test_sensors_not_supported.json](./config/test_sensors_not_supported.json). However, the implementation checks for invalid configurations and shows dedicated warnings.
