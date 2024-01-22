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

        # Map object types to corresponding functions
        self.object_type_map = {
            'vehicle': self.process_vehicle,
            'blueprint': self.process_blueprint,
            'sensor': self.process_sensor
        }
        self.world_frame = "carla_map"
        self.players = []
        self.vehicles_sensors = []
        self.global_sensors = []
        self.attached_vehicle_id = None

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

        self.blueprints = json_actors.get('blueprints', [])  # Read blueprints and store them

        for obj in json_actors.get('objects', []):  # Iterate through all objects
            self.process_object(obj, True)
        self.loginfo("All objects spawned.")

    def process_object(self, obj, top_layer):
        if top_layer:
            self.parent_obj_id = obj["id"]
            if "spawn_point" in obj:
                self.spawn_point_parent = obj["spawn_point"]
        # Get the corresponding function and call it
        func = self.object_type_map.get(obj["type"].split('.')[0], None)
        if func:
            func(obj, top_layer)

    def process_vehicle(self, vehicle, top_layer = True):
        # Spawn Vehicle
        if self.spawn_sensors_only is True:
            # spawn sensors of already spawned vehicles
            try:
                carla_id = vehicle["carla_id"]
            except KeyError as e:
                self.logerr(
                    "Could not spawn sensors of vehicle {}, its carla ID is not known.".format(vehicle["id"]))
            # spawn the vehicle's sensors
            self.setup_sensors(vehicle["sensors"], carla_id)
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

                response_id = self.spawn_object(spawn_object_request)
                self.attached_vehicle_id = response_id
                if response_id != -1:
                    player_spawned = True
                    self.players.append(response_id)
                    # Set up the sensors
                    try:
                        # Recursively process child objects:
                        for child in vehicle.get('children', []):
                            self.process_object(child, False)
                    except KeyError:
                        self.logwarn(
                            "Object (type='{}', id='{}') has no 'sensors' field in his config file, none will be spawned.".format(spawn_object_request.type, spawn_object_request.id))
            self.attached_vehicle_id = None
                
    def process_blueprint(self, obj, top_layer):
        # Process blueprint object and its chrildren
        for blueprint in self.blueprints:
            if blueprint['id'] == obj['type'].split('.')[1]:

                # Get spwan point of blueprint from config file
                spawn_point_blueprint = self.create_spawn_point(
                        obj["spawn_point"]["x"],
                        obj["spawn_point"]["y"],
                        obj["spawn_point"]["z"],
                        obj["spawn_point"]["roll"],
                        obj["spawn_point"]["pitch"],
                        obj["spawn_point"]["yaw"]
                    )
                
                # initialize static transform between "carla_map" frame and spawn point of group
                static_transform = geometry_msgs.msg.TransformStamped()
                if ROS_VERSION == 1:
                    broadcaster = tf2_ros.StaticTransformBroadcaster()
                    static_transform.header.stamp = rospy.Time.now()
                elif ROS_VERSION == 2:
                    broadcaster = tf2_ros.StaticTransformBroadcaster(self)
                    static_transform.header.stamp = self.get_clock().now().to_msg()
                if top_layer:
                    static_transform.header.frame_id = self.world_frame 
                elif not top_layer:
                    static_transform.header.frame_id = self.parent_obj_id
                static_transform.child_frame_id = obj["id"]
                static_transform.transform.translation.x = spawn_point_blueprint.position.x
                static_transform.transform.translation.y = spawn_point_blueprint.position.y
                static_transform.transform.translation.z = spawn_point_blueprint.position.z
                static_transform.transform.rotation.x = spawn_point_blueprint.orientation.x
                static_transform.transform.rotation.y = spawn_point_blueprint.orientation.y
                static_transform.transform.rotation.z = spawn_point_blueprint.orientation.z
                static_transform.transform.rotation.w = spawn_point_blueprint.orientation.w
                broadcaster.sendTransform(static_transform) 

                if "physical_object" in blueprint:
                    # Spawn blueprint when it is a physical_object
                    spawn_object_request = roscomp.get_service_request(SpawnObject)
                    spawn_object_request.type = blueprint["physical_object"]     
                    spawn_object_request.id = self.parent_obj_id
                    spawn_object_request.attach_to = 0
                    spawn_object_request.random_pose = False

                    player_spawned = False
                    while not player_spawned and roscomp.ok():
                        spawn_object_request.transform = spawn_point_blueprint

                        response_id = self.spawn_object(spawn_object_request)
                        self.attached_vehicle_id = response_id
                        if response_id != -1:
                            player_spawned = True
                            self.players.append(response_id)
                            # Set up the sensors
                            try:
                                # Recursively process child objects:
                                for child in blueprint.get('children', []):
                                    self.process_object(child, False)
                            except KeyError:
                                self.logwarn(
                                    "Object (type='{}', id='{}') has no 'sensors' field in his config file, none will be spawned.".format(spawn_object_request.type, spawn_object_request.id))
                    self.attached_vehicle_id = None
                else:
                    for child in blueprint.get('children', []):
                        self.process_object(child, False)
            self.spawn_point_parent.clear()
                
    def process_sensor(self, sensor, top_layer):
        """
        Create the sensors defined by the user and attach them to the vehicle
        (or not if global sensor)
        :param sensors: list of sensors
        :param attached_vehicle_id: id of vehicle to attach the sensors to
        :return actors: list of ids of objects created
        """
        sensor_names = []
        try:
            sensor_type = str(sensor.pop("type"))
            sensor_id = str(sensor.pop("id"))

            sensor_name = sensor_type + "/" + sensor_id
            if sensor_name in sensor_names:
                raise NameError
            sensor_names.append(sensor_name)

            if self.attached_vehicle_id is None and "pseudo" not in sensor_type:
                spawn_point = sensor.pop("spawn_point")
                sensor_transform = self.create_spawn_point(
                    spawn_point.pop("x") + self.spawn_point_parent["x"],
                    spawn_point.pop("y") + self.spawn_point_parent["y"],
                    spawn_point.pop("z") + self.spawn_point_parent["z"],
                    spawn_point.pop("roll", 0.0) + self.spawn_point_parent["roll"],
                    spawn_point.pop("pitch", 0.0) + self.spawn_point_parent["pitch"],
                    spawn_point.pop("yaw", 0.0) + self.spawn_point_parent["yaw"])
            else:
                # if sensor attached to a vehicle, or is a 'pseudo_actor', allow default pose
                spawn_point = sensor.pop("spawn_point", 0)
                if spawn_point == 0:
                    sensor_transform = self.create_spawn_point(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                else:
                    sensor_transform = self.create_spawn_point(
                    spawn_point.pop("x"),
                    spawn_point.pop("y"),
                    spawn_point.pop("z"),
                    spawn_point.pop("roll", 0.0),
                    spawn_point.pop("pitch", 0.0),
                    spawn_point.pop("yaw", 0.0))

            spawn_object_request = roscomp.get_service_request(SpawnObject)
            spawn_object_request.type = sensor_type
            spawn_object_request.id = sensor_id
            spawn_object_request.attach_to = self.attached_vehicle_id if self.attached_vehicle_id is not None else 0
            spawn_object_request.transform = sensor_transform
            spawn_object_request.random_pose = False  # never set a random pose for a sensor

            attached_objects = []
            for attribute, value in sensor.items():
                if attribute == "attached_objects":
                    for attached_object in sensor["attached_objects"]:
                        attached_objects.append(attached_object)
                    continue
                spawn_object_request.attributes.append(
                    KeyValue(key=str(attribute), value=str(value)))

            response_id = self.spawn_object(spawn_object_request)

            if response_id == -1:
                raise RuntimeError(response.error_string)

            if attached_objects:
                # spawn the attached objects
                self.process_object(attached_objects, response_id)

            if self.attached_vehicle_id is None:
                self.global_sensors.append(response_id)
            else:
                self.vehicles_sensors.append(response_id)

        except KeyError as e:
            self.logerr(
                "Sensor {} will not be spawned, the mandatory attribute {} is missing".format(sensor_name, e))

        except RuntimeError as e:
            self.logerr(
                "Sensor {} will not be spawned: {}".format(sensor_name, e))

        except NameError:
            self.logerr("Sensor rolename '{}' is only allowed to be used once. The second one will be ignored.".format(
                sensor_id))

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