# CARLA Spawn Objects

Find the official documentation about the CARLA Spawn Objects package [__here__](https://carla.readthedocs.io/projects/ros-bridge/en/latest/carla_spawn_objects/). The following README contains information about OpenADS specific changes regarding the `carla_spawn_objects` package.

---

## Object Configuration

Vehicles and sensors can be defined using a configuration file (see [objects.json](./config/objects.json)). Besides the two object type `vehicle` and `sensor`, two _placeholder_ entities are added, namely the `group` and `blueprint` type. All four entities are further described in the following.

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

A sensor represents a CARLA sensor to record data. Sensors can act globally or are attached to actors (vehicles and walkers). The sensor configuration is consistent with the original `carla_spawn_objects` package. However, there are some important remarks for the configuration of sensors:
- the type has to start with `sensor.`
- a sensor does not have any children

Thus, a <sensor_configuration> can be configured as:
```
{
  "id": "<sensor_name>",
  "type": "sensor.<sensor_type>",
  ...
}
```

### Groups

A group does not represent any CARLA entity, but acts as a placeholder which can aggregate different other entities within a common coordinate system. Thus, children of a group are recursively configured based on the groups transformation. However, there are some important remarks for the configuration of groups:
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

A blueprint does not represent any CARLA entity, but acts as a placeholder. A blueprint is defined in a special blueprint-section at the bottom of the configuration file. A blueprint can represent a vehicle, sensor, group, but not another blueprint. 
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

Those defined blueprints can then be used within the objects-section in different configurations. Either on top-level, as group child or vehicle child a blueprint can be configured within the objects-section. A specific blueprint can be selected by using the type `blueprint.<blueprint_id>`. 

Thus, the prefix `blueprint.` is required to configure a <blueprint_configuration>:
``````
{
  "id": "<blueprint_name>",
  "type": "blueprint.<blueprint_id>",
}
``````


## Testcases

All entities vehicle, sensor, group and blueprint can contain other entities. Therefore, 16 different configurations can be posed, where only some of them are valid with respect to the current implementation. The following table gives an overview of all supported cases. 

| entity A (column) contain entity B (row) 	| **vehicle** 	| **sensor**  	| **group**   	| **blueprint** 	|
|--------------------------	|-------------	|-------------	|-------------	|---------------	|
| **vehicle**              	| _invalid_ 	| _invalid_ 	| _invalid_ 	| valid         	|
| **sensor**               	| valid       	| _invalid_ 	| valid       	| valid         	|
| **group**                	| valid       	| _invalid_ 	| valid       	| valid         	|
| **blueprint**            	| valid       	| _invalid_ 	| valid       	| _invalid_   	|


In addition all valid configurations are somehow contained in the demo file [sensors_check_supported_config.json](./config/sensors_check_supported_config.json), whereas all invalid configurations are tested in [sensors_check_not_supported_config.json](./config/sensors_check_not_supported_config.json). However, the implementation checks for invalid configurations and shows dedicated warnings.