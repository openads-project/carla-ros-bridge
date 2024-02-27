# CARLA Spawn Objects

Find the official documentation about the CARLA Spawn Objects package [__here__](https://carla.readthedocs.io/projects/ros-bridge/en/latest/carla_spawn_objects/). In the following an ika specific README is provided.


- observe the example [objects.json](../../config/objects.json)
- (TODO) table with all possible configurations
  
  -> Testcatalog of all combinations as json files
## Vehicles

- a vehicle needs to be defined on top level
  - a vehicle can not be attached to other vehicles
- a vehicle can contain multiple sensors/children
  - a child can be
    - a sensor (tested)
    - a blueprint (tested)
    - a group (tested)

## Groups

- a group can be defined on top level
- a group can be defined in blueprints or vehicles
- a group can contain multiple children
  - a child can be
    - a sensor (tested)
    - a blueprint (tested)
    - a group (not tested)

## Blueprints

- a blueprint can be defined in the blueprint section
- a blueprint can be used in other blueprints
- a blueprint can be used as a group child
- a blueprint can be used as a vehicle child
- a blueprint can be of type
  - group (tested)
  - sensor (not tested)
  - vehicle (not tested)