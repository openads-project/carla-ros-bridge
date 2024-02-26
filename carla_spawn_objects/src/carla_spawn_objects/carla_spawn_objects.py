#!/usr/bin/env python
#
# Copyright (c) 2019-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
base class for spawning objects (carla actors and pseudo_actors) in ROS

Gets config file from ros parameter ~objects_definition_file and spawns corresponding objects
through ROS service /carla/spawn_object.

Looks for an initial spawn point first in the launchfile, then in the config file, and
finally ask for a random one to the spawn service.

"""

import json
import math
import os

from transforms3d.euler import euler2quat

import ros_compatibility as roscomp
ROS_VERSION = roscomp.get_ros_version()
if ROS_VERSION == 1:
    import rospy
from ros_compatibility.exceptions import *
from ros_compatibility.node import CompatibleNode

from carla_msgs.msg import CarlaActorList
from carla_msgs.srv import SpawnObject, DestroyObject
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Pose
import geometry_msgs.msg
import tf2_ros

# ==============================================================================
# -- CarlaSpawnObjects ------------------------------------------------------------
# ==============================================================================


class CarlaSpawnObjects(CompatibleNode):

    """
    Handles the spawning of the ego vehicle and its sensors

    Derive from this class and implement method sensors()
    """

    def __init__(self):
        super(CarlaSpawnObjects, self).__init__('carla_spawn_objects')

        self.objects_definition_file = self.get_param('objects_definition_file', '')
        self.spawn_sensors_only = self.get_param('spawn_sensors_only', False)

        # map object types to corresponding processing functions
        self.object_type_map = {
            'vehicle': self.process_vehicle,
            'walker': self.process_vehicle,
            'sensor': self.process_sensor,
            'group': self.process_group,
            'blueprint': self.process_blueprint
        }
        self.world_frame = "carla_map"

        # managing lists of spawned entities
        self.players = []
        self.vehicles_sensors = []
        self.global_sensors = []
        self.sensor_names = []
        self.group_names = []

        self.spawn_object_service = self.new_client(SpawnObject, "/carla/spawn_object")
        self.destroy_object_service = self.new_client(DestroyObject, "/carla/destroy_object")

    def spawn_object(self, spawn_object_request):
        response_id = -1
        response = self.call_service(self.spawn_object_service, spawn_object_request, spin_until_response_received=True)
        response_id = response.id
        if response_id != -1:
            self.loginfo("Object (type='{}', id='{}') spawned successfully as {}.".format(
                spawn_object_request.type, spawn_object_request.id, response_id))
        else:
            self.logwarn("Error while spawning object (type='{}', id='{}').".format(
                spawn_object_request.type, spawn_object_request.id))
            raise RuntimeError(response.error_string)
        return response_id

    def spawn_objects(self):
        """
        Spawns the objects

        Either at a given spawnpoint or at a random Carla spawnpoint

        :return:
        """
        # Read sensors from file
        if not self.objects_definition_file or not os.path.exists(self.objects_definition_file):
            raise RuntimeError(
                "Could not read object definitions from {}".format(self.objects_definition_file))
        with open(self.objects_definition_file) as handle:
            json_actors = json.loads(handle.read())

        self.blueprints = json_actors.get('blueprints', [])  # Read blueprints 
        self.objects = json_actors.get('objects', [])  # Read objects

        global_sensors = [obj for obj in self.objects if obj['type'].split('.')[0] == 'sensor']
        global_vehicles = [obj for obj in self.objects if obj['type'].split('.')[0] == 'vehicle' or obj['type'].split('.')[0] == 'walker']
        global_groups = [obj for obj in self.objects if obj['type'].split('.')[0] == 'group']


        found_sensor_actor_list = any(sensor['id'] == 'sensor.pseudo.actor_list' for sensor in global_sensors)

        if self.spawn_sensors_only is True and found_sensor_actor_list is False:
            raise RuntimeError("Parameter 'spawn_sensors_only' enabled, " +
                               "but 'sensor.pseudo.actor_list' is not instantiated, add it to your config file.")

        # iterate through all top-level sensors
        for global_sensor in global_sensors:
            self.process_sensor(global_sensor, None)

        if self.spawn_sensors_only is True:
            # get existing carla vehicle id from topic /carla/actor_list for all vehicles listed in config file
            actor_info_list = self.wait_for_message("/carla/actor_list", CarlaActorList)
            for vehicle in global_vehicles:
                for actor_info in actor_info_list.actors:
                    if actor_info.type == vehicle["type"] and actor_info.rolename == vehicle["id"]:
                        vehicle["carla_id"] = actor_info.id

        # iterate through all top-level vehicles
        for vehicle in global_vehicles:
            self.process_vehicle(vehicle, None)

        # iterate through all top-level groups
        for group in global_groups:
            self.process_group(group, None)

        self.loginfo("All objects spawned.")

    def process_vehicle(self, vehicle, parent):
        """
        Create the vehicle defined by the input dict
        :param vehicle: vehicle input dict
        :param parent: id of attached parent
        """
        if self.spawn_sensors_only is True:
            # spawn sensors of non-ros spawned vehicles
            try:
                vehicle["carla_id"]
            except KeyError as e:
                self.logerr(
                    "Could not spawn sensors of vehicle {}, its carla ID is not known.".format(vehicle["id"]))
            # spawn the vehicle's sensors
            for sensor in vehicle["sensors"]:
                self.process_sensor(sensor, vehicle)
        else:
            spawn_object_request = roscomp.get_service_request(SpawnObject)
            spawn_object_request.type = vehicle["type"]
            spawn_object_request.id = vehicle["id"]
            spawn_object_request.attach_to = 0
            spawn_object_request.random_pose = False

            spawn_point = None

            # check if there's a spawn_point corresponding to this vehicle
            spawn_point_param = self.get_param("spawn_point_" + vehicle["id"], None)
            spawn_param_used = False
            if (spawn_point_param is not None):
                # try to use spawn_point from parameters
                spawn_point = self.check_spawn_point_param(spawn_point_param)
                if spawn_point is None:
                    self.logwarn("{}: Could not use spawn point from parameters, ".format(vehicle["id"]) +
                                    "the spawn point from config file will be used.")
                else:
                    self.loginfo("Spawn point from ros parameters")
                    spawn_param_used = True

            if "spawn_point" in vehicle and spawn_param_used is False:
                # get spawn point from config file
                try:
                    spawn_point = self.create_spawn_point(
                        vehicle["spawn_point"]["x"],
                        vehicle["spawn_point"]["y"],
                        vehicle["spawn_point"]["z"],
                        vehicle["spawn_point"]["roll"],
                        vehicle["spawn_point"]["pitch"],
                        vehicle["spawn_point"]["yaw"]
                    )
                    self.loginfo("Spawn point from configuration file")
                except KeyError as e:
                    self.logerr("{}: Could not use the spawn point from config file, ".format(vehicle["id"]) +
                                "the mandatory attribute {} is missing, a random spawn point will be used".format(e))

            if spawn_point is None:
                # pose not specified, ask for a random one in the service call
                self.loginfo("Spawn point selected at random")
                spawn_point = Pose()  # empty pose
                spawn_object_request.random_pose = True

            player_spawned = False
            while not player_spawned and roscomp.ok():
                spawn_object_request.transform = spawn_point

                vehicle['response_id'] = self.spawn_object(spawn_object_request)
                if vehicle['response_id'] != -1:
                    player_spawned = True
                    self.players.append(vehicle['response_id'])
                        
                    # recursively process sensor objects:
                    for object in vehicle.get('sensors', []):
                        self.process_object(object, vehicle)
                    # recursively process child objects:
                    for object in vehicle.get('children', []):
                        self.process_object(object, vehicle)

    def process_sensor(self, sensor, parent):
        """
        Create the sensor defined by the input dict
        :param sensor: sensor input dict
        :param parent: id of attached parent
        """
        if not roscomp.ok():
            return

        try:
            sensor_type = str(sensor.pop("type"))
            sensor_id = str(sensor.pop("id"))

            # check if sensor name already exists
            sensor_name = sensor_type + "/" + sensor_id 
            if sensor_name in self.sensor_names:        # TODO: could be sensor with same id on different levels
                raise NameError
            self.sensor_names.append(sensor_name)

            if parent is None and "pseudo" not in sensor_type:
                spawn_point = sensor.pop("spawn_point")
                sensor['transform'] = self.create_spawn_point(
                    spawn_point.pop("x"),
                    spawn_point.pop("y"),
                    spawn_point.pop("z"),
                    spawn_point.pop("roll", 0.0),
                    spawn_point.pop("pitch", 0.0),
                    spawn_point.pop("yaw", 0.0)
                )
            else:
                # if sensor attached to a parent, or is a 'pseudo_actor', allow default pose
                spawn_point = sensor.pop("spawn_point", 0)
                if spawn_point == 0:
                    sensor['transform'] = self.create_spawn_point(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                else:
                    sensor['transform'] = self.create_spawn_point(
                    spawn_point.pop("x"),
                    spawn_point.pop("y"),
                    spawn_point.pop("z"),
                    spawn_point.pop("roll", 0.0),
                    spawn_point.pop("pitch", 0.0),
                    spawn_point.pop("yaw", 0.0))

            # Consider parent object transformations
            sensor['attached_vehicle_id'] = 0
            if parent is not None:
                
                if parent['type'] == 'vehicle' and 'attached_vehicle_id' in parent:
                    raise RuntimeError("Sensor {} will not be spawned, the parent vehicle {} is already attached to another vehicle.".format(sensor_name, parent['id']))

                elif parent['type'] == 'vehicle':
                    sensor['attached_vehicle_id'] = parent['id']

                elif 'attached_vehicle_id' in parent:
                    sensor['attached_vehicle_id'] = parent['attached_vehicle_id']
                    sensor['transform'] = self.combine_spawn_point(parent['transform'], sensor['transform'])

            spawn_object_request = roscomp.get_service_request(SpawnObject)
            spawn_object_request.type = sensor_type
            spawn_object_request.id = sensor_id
            spawn_object_request.attach_to = sensor['attached_vehicle_id']
            spawn_object_request.transform = sensor['transform']
            spawn_object_request.random_pose = False  # never set a random pose for a sensor

            attached_objects = []
            for attribute, value in sensor.items():
                if attribute == "attached_objects":
                    for attached_object in sensor["attached_objects"]:
                        attached_objects.append(attached_object)
                    continue
                spawn_object_request.attributes.append(
                    KeyValue(key=str(attribute), value=str(value)))

            sensor['response_id'] = self.spawn_object(spawn_object_request)

            if sensor['response_id'] == -1:
                raise RuntimeError(response.error_string)

            # spawn the attached objects
            for attached_object in attached_objects:
                self.process_object(attached_object, sensor)

            if parent is None:
                self.global_sensors.append(sensor['response_id'])
            else:
                self.vehicles_sensors.append(sensor['response_id'])

        except KeyError as e:
            self.logerr(
                "Sensor {} will not be spawned, the mandatory attribute {} is missing".format(sensor_name, e))
            return

        except RuntimeError as e:
            self.logerr(
                "Sensor {} will not be spawned: {}".format(sensor_name, e))
            return

        except NameError:
            self.logerr("Sensor rolename '{}' is only allowed to be used once. The second one will be ignored.".format(
                sensor_id))
            return

    def process_group(self, group, parent):

        if not roscomp.ok():
            return

        try:
            group_name = str(group.pop("id"))

            # check if group name already exists
            if group_name in self.group_names:
                raise NameError
            self.group_names.append(group_name)       # TODO: could be group with same id on different levels

            if parent is None:
                spawn_point = group.pop("spawn_point")
                group['transform'] = self.create_spawn_point(
                    spawn_point.pop("x"),
                    spawn_point.pop("y"),
                    spawn_point.pop("z"),
                    spawn_point.pop("roll", 0.0),
                    spawn_point.pop("pitch", 0.0),
                    spawn_point.pop("yaw", 0.0)
                )
            else:
                # if group attached to a parent allow default pose
                spawn_point = group.pop("spawn_point", 0)
                if spawn_point == 0:
                    group['transform'] = self.create_spawn_point(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                else:
                    group['transform'] = self.create_spawn_point(
                    spawn_point.pop("x"),
                    spawn_point.pop("y"),
                    spawn_point.pop("z"),
                    spawn_point.pop("roll", 0.0),
                    spawn_point.pop("pitch", 0.0),
                    spawn_point.pop("yaw", 0.0))

            # Consider parent object transformations
            group['attached_vehicle_id'] = 0
            if parent is not None:
                
                if parent['type'] == 'vehicle' and 'attached_vehicle_id' in parent:
                    raise RuntimeError("Sensor {} will not be spawned, the parent vehicle {} is already attached to another vehicle.".format(sensor_name, parent['id']))

                elif parent['type'] == 'vehicle':
                    group['attached_vehicle_id'] = parent['id']

                elif 'attached_vehicle_id' in parent:
                    group['attached_vehicle_id'] = parent['attached_vehicle_id']
                    group['transform'] = self.combine_spawn_point(parent['transform'], group['transform'])

            if 'physical_object' in group:
                spawn_object_request = roscomp.get_service_request(SpawnObject)
                spawn_object_request.type = group["physical_object"]    
                spawn_object_request.id = group_name
                spawn_object_request.attach_to = group['attached_vehicle_id']
                spawn_object_request.transform = group['transform']
                spawn_object_request.random_pose = False
            
                group_spawned = False
                while not group_spawned and roscomp.ok():

                    group['response_id'] = self.spawn_object(spawn_object_request)
                    if group['response_id'] != -1:
                        group_spawned = True
                        self.players.append(group['response_id'])
            
            # spawn the child objects
            for child in group['children']:
                self.process_object(child, group)

        except RuntimeError as e:
            self.logerr(
                "Group {} will not be spawned: {}".format(group_name, e))
            return

        except NameError:
            self.logerr("Group name '{}' is only allowed to be used once. The second one will be ignored.".format(
                group_name))
            return


        # initialize static transform for group         # TODO: check if that code is needed
        #static_transform = geometry_msgs.msg.TransformStamped()
        #if ROS_VERSION == 1:
        #    broadcaster = tf2_ros.StaticTransformBroadcaster()
        #    static_transform.header.stamp = rospy.Time.now()
        #elif ROS_VERSION == 2:
        #    broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        #    static_transform.header.stamp = self.get_clock().now().to_msg()

        #if parent is None:
        #    static_transform.header.frame_id = self.world_frame
        #else:
        #    static_transform.header.frame_id = parent['id']

        #static_transform.child_frame_id = group_name
        #static_transform.transform.translation.x = group['spawn_point'].position.x
        #static_transform.transform.translation.y = group['spawn_point'].position.y
        #static_transform.transform.translation.z = group['spawn_point'].position.z
        #static_transform.transform.rotation.x = group['spawn_point'].orientation.x
        #static_transform.transform.rotation.y = group['spawn_point'].orientation.y
        #static_transform.transform.rotation.z = group['spawn_point'].orientation.z
        #static_transform.transform.rotation.w = group['spawn_point'].orientation.w
        #broadcaster.sendTransform(static_transform) 

    def process_blueprint(self, object, parent):
        # take blueprint and add object information
        for blueprint in self.blueprints:
            if blueprint['id'] != object['type'].split('.')[1]: continue

            blueprint["id"] = object["id"]
            blueprint["spawn_point"] = object["spawn_point"]

            self.process_object(blueprint, parent)
    
    def process_object(self, obj, parent):
     
        # Get the corresponding function and call it
        func = self.object_type_map.get(obj["type"].split('.')[0], None)
        if func:
            func(obj, parent)
        else:
            self.logwarn(
                    "Object with type {} is not a vehicle, a walker or a sensor, ignoring".format(obj["type"]))

    def create_spawn_point(self, x, y, z, roll, pitch, yaw):
        spawn_point = Pose()
        spawn_point.position.x = x
        spawn_point.position.y = y
        spawn_point.position.z = z
        quat = euler2quat(math.radians(roll), math.radians(pitch), math.radians(yaw))

        spawn_point.orientation.w = quat[0]
        spawn_point.orientation.x = quat[1]
        spawn_point.orientation.y = quat[2]
        spawn_point.orientation.z = quat[3]
        return spawn_point

    def combine_spawn_point(self, base, shift):

        base.position.x += shift.position.x
        base.position.y += shift.position.y
        base.position.z += shift.position.z

        base_orientation = quat2euler([base.orientation.w,
                                base.orientation.x,
                                base.orientation.y,
                                base.orientation.z])

        shift_orientation = quat2euler([shift.orientation.w,
        shift.orientation.x,
        shift.orientation.y,
        shift.orientation.z])

        base_orientation[0] += shift_orientation[0]
        base_orientation[1] += shift_orientation[1]
        base_orientation[2] += shift_orientation[2]

        quat = euler2quat(base_orientation[0], base_orientation[1], base_orientation[2])

        base.orientation.w = quat[0]
        base.orientation.x = quat[1]
        base.orientation.y = quat[2]
        base.orientation.z = quat[3]
        
        return base

    def check_spawn_point_param(self, spawn_point_parameter):
        components = spawn_point_parameter.split(',')
        if len(components) != 6:
            self.logwarn("Invalid spawnpoint '{}'".format(spawn_point_parameter))
            return None
        spawn_point = self.create_spawn_point(
            float(components[0]),
            float(components[1]),
            float(components[2]),
            float(components[3]),
            float(components[4]),
            float(components[5])
        )
        return spawn_point

    def destroy(self):
        """
        destroy all the players and sensors
        """
        self.loginfo("Destroying spawned objects...")
        try:
            # destroy vehicles sensors
            for actor_id in self.vehicles_sensors:
                destroy_object_request = roscomp.get_service_request(DestroyObject)
                destroy_object_request.id = actor_id
                self.call_service(self.destroy_object_service,
                                  destroy_object_request, timeout=0.5, spin_until_response_received=True)
                self.loginfo("Object {} successfully destroyed.".format(actor_id))
            self.vehicles_sensors = []

            # destroy global sensors
            for actor_id in self.global_sensors:
                destroy_object_request = roscomp.get_service_request(DestroyObject)
                destroy_object_request.id = actor_id
                self.call_service(self.destroy_object_service,
                                  destroy_object_request, timeout=0.5, spin_until_response_received=True)
                self.loginfo("Object {} successfully destroyed.".format(actor_id))
            self.global_sensors = []

            # destroy player
            for player_id in self.players:
                destroy_object_request = roscomp.get_service_request(DestroyObject)
                destroy_object_request.id = player_id
                self.call_service(self.destroy_object_service,
                                  destroy_object_request, timeout=0.5, spin_until_response_received=True)
                self.loginfo("Object {} successfully destroyed.".format(player_id))
            self.players = []
        except ServiceException:
            self.logwarn(
                'Could not call destroy service on objects, the ros bridge is probably already shutdown')

# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main(args=None):
    """
    main function
    """
    roscomp.init("spawn_objects", args=args)
    spawn_objects_node = None
    try:
        spawn_objects_node = CarlaSpawnObjects()
        roscomp.on_shutdown(spawn_objects_node.destroy)
    except KeyboardInterrupt:
        roscomp.logerr("Could not initialize CarlaSpawnObjects. Shutting down.")

    if spawn_objects_node:
        try:
            spawn_objects_node.spawn_objects()
            try:
                spawn_objects_node.spin()
            except (ROSInterruptException, ServiceException, KeyboardInterrupt):
                pass
        except (ROSInterruptException, ServiceException, KeyboardInterrupt):
            spawn_objects_node.logwarn(
                "Spawning process has been interrupted. There might be actors that have not been destroyed properly")
        except RuntimeError as e:
            roscomp.logfatal("Exception caught: {}".format(e))
        finally:
            roscomp.shutdown()


if __name__ == '__main__':
    main()
