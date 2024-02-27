# CARLA Spawn Objects

Find the official documentation about the CARLA Spawn Objects package [__here__](https://carla.readthedocs.io/projects/ros-bridge/en/latest/carla_spawn_objects/). 

In general, there are four different entities within the ika fork, more explained in the following:

## Configuration Entities

### Vehicles
- has to be defined on top level
- type has to start with `vehicle.`
- a vehicle can contain multiple `children` of type sensor, group, blueprint

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

### Sensors
- type has to start with `sensor.`
- a sensor does not have any children

Thus, a <sensor_configuration> can be configured as:
```
{
  "id": "<sensor_name>",
  "type": "sensor.<sensor_model>",
  ...
}
```

### Groups
- type has to start with `group.`
- a group can contain multiple `children` of type sensor, group, blueprint
- a group can also represent an _optional_ physical static object

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
- type has to start with `blueprint.`

Thus, a <blueprint_configuration> can be configured as:
``````
{
  "id": "<blueprint_name>",
  "type": "blueprint.<blueprint_definition>",
}
``````

In addition the used type has to be defined in a separate blueprint section at the end of the `objects.json` file. Here we can defined vehicles, sensors, and groups as blueprint. Make sure that the `id` is used as `blueprint_definition` in the configuration above.
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

## Testcases

Based on the four entities 16 different configurations exist, where only some of them are valid. The following table gives an overview of all supported cases. In addition all configurations are contained in the two files [sensors_check_supported_config.json](./config/sensors_check_supported_config.json) and [sensors_check_not_supported_config.json](./config/sensors_check_not_supported_config.json)

| entity defined in entity 	| **vehicle** 	| **sensor**  	| **group**   	| **blueprint** 	|
|--------------------------	|-------------	|-------------	|-------------	|---------------	|
| **vehicle**              	| _not valid_ 	| _not valid_ 	| _not valid_ 	| valid         	|
| **sensor**               	| valid       	| _not valid_ 	| valid       	| valid         	|
| **group**                	| valid       	| _not valid_ 	| valid       	| valid         	|
| **blueprint**            	| valid       	| _not valid_ 	| valid       	| _not valid_   	|
