# CARLA Spawn Objects

Find the official documentation about the CARLA Spawn Objects package [__here__](https://carla.readthedocs.io/projects/ros-bridge/en/latest/carla_spawn_objects/). In the following an ika specific README is provided.


- observe the example [objects.json](../../config/objects.json)
- (TODO) table with all possible configurations
  
## Vehicles

- a vehicle needs to be defined on top level
- a vehicle can not be attached to other vehicles
  
## Groups

- a group have one or multiple children
- a child can be
  - a sensor (tested)
  - a blueprint (tested)
  - a group (not tested)

## Blueprints

- a blueprint can be defined in the blueprint section
- a blueprint can be used in other blueprints
- a blueprint can be used in the objects section
- a blueprint can be of type
  - group (tested)
  - sensor (not tested)
  - vehicle (not tested)

# Open ToDos: 

- check available ros tf's
- check correct spawnpoints