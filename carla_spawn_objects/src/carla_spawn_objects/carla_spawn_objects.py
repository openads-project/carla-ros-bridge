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

import copy
import json
import math
import os
import time

import ros_compatibility as roscomp
from ros_compatibility.exceptions import *
from ros_compatibility.node import CompatibleNode
ROS_VERSION = roscomp.get_ros_version()
if ROS_VERSION == 1:
    import rospy
else:
    import rclpy
    from rclpy.duration import Duration

from carla_msgs.msg import CarlaActorList
from carla_msgs.srv import SpawnObject, DestroyObject
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Pose
import geometry_msgs.msg
import tf2_geometry_msgs
import tf2_ros
from transforms3d.euler import quat2euler, euler2quat
import pyproj


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
        self.objects_directory = self.get_param('objects_directory', '')
        self.blueprints_directory = self.get_param('blueprints_directory', '')
        self.spawn_sensors_only = self.get_param('spawn_sensors_only', False)

        # map object types to corresponding processing functions
        self.object_type_map = {
            'vehicle': self.process_vehicle,
            'walker': self.process_vehicle,
            'sensor': self.process_sensor,
            'actor': self.process_sensor,
            'group': self.process_group,
            'blueprint': self.process_blueprint
        }
        self.world_frame = "carla_map"
        self.tf_wait_timeout = 15.0
        self.tf_buffer = tf2_ros.Buffer()
        if ROS_VERSION == 1:
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        else:
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)
        self._utm_proj_cache = {}

        # lists of spawned entities
        self.players = []
        self.vehicles_sensors = []
        self.global_sensors = []

        self.object_names = []

        # setup services
        self.spawn_object_service = self.new_client(SpawnObject, "/carla/spawn_object")
        self.destroy_object_service = self.new_client(DestroyObject, "/carla/destroy_object")

    def wgs84_to_carla_spawn_point(self, lat, lon, alt, roll=0.0, pitch=0.0, yaw=0.0):
        """
        Convert WGS84 coordinates to CARLA coordinates
        """

        north = lat >= 0.0
        zone = int(math.floor((lon + 180.0) / 6.0) + 1)
        proj_key = (zone, north)
        if proj_key not in self._utm_proj_cache:
            proj_args = {'proj': 'utm', 'zone': zone, 'ellps': 'WGS84', 'preserve_units': False}
            if not north:
                proj_args['south'] = True
            self._utm_proj_cache[proj_key] = pyproj.Proj(**proj_args)
        proj = self._utm_proj_cache[proj_key]
        utm_x, utm_y = proj(lon, lat)

        frame_id = "utm_{}{}".format(zone, "N" if north else "S")

        utm_pose = geometry_msgs.msg.PoseStamped()
        utm_pose.header.frame_id = frame_id
        utm_pose.header.stamp = roscomp.ros_timestamp(sec=0.0, from_sec=True)
        utm_pose.pose = self.create_spawn_point(utm_x, utm_y, alt, roll, pitch, yaw)

        try:
            self.loginfo("Waiting up to {}s for transform '{}' -> '{}'.".format(
                self.tf_wait_timeout, frame_id, self.world_frame))
            self._wait_for_transform(self.world_frame, frame_id, utm_pose.header.stamp)
            carla_pose = self.tf_buffer.transform(
                utm_pose, self.world_frame)
            return carla_pose.pose

        except Exception as e:
            self.logerr("Could not transform spawn point from '{}' to '{}': {}".format(frame_id, self.world_frame, e))
            raise

    def _wait_for_transform(self, target_frame, source_frame, stamp):

        timeout = time.monotonic() + self.tf_wait_timeout
        while roscomp.ok():
            if self.tf_buffer.can_transform(target_frame, source_frame, stamp, Duration(seconds=0.05)):
                return
            if time.monotonic() >= timeout:
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        raise RuntimeError("Timed out waiting for transform")

    def resolve_spawn_point(self, spawn_point):
        """
        Build a CARLA-map-frame Pose from a spawn point definition.

        Supports two formats:
        - spawn points already in CARLA coordinates (x/y/[z/roll/pitch/yaw])
        - spawn points given in WGS84 (lat/lon/[alt/roll/pitch/yaw])
        """
        roll = spawn_point.get("roll", 0.0)
        pitch = spawn_point.get("pitch", 0.0)
        yaw = spawn_point.get("yaw", 0.0)

        if 'lat' in spawn_point and 'lon' in spawn_point:

            return self.wgs84_to_carla_spawn_point(
                spawn_point['lat'],
                spawn_point['lon'],
                spawn_point.get('alt', 0.0),
                roll,
                pitch,
                yaw)

        if 'x' in spawn_point and 'y' in spawn_point:

            return self.create_spawn_point(
                spawn_point["x"],
                spawn_point["y"],
                spawn_point.get("z", 0.0),
                roll,
                pitch,
                yaw)

    def spawn_object(self, spawn_object_request):
        """
        Spawns the object defined by the object input request via ROS service
        :param spawn_object_request: object input request
        :return: response id
        """

        attr_preview = ", ".join(
            ["{}={}".format(attr.key, attr.value) for attr in spawn_object_request.attributes]
        )
        self.loginfo(
            "SpawnObject request -> type='{}', id='{}', attach_to={}, random_pose={}, attributes=[{}]".format(
                spawn_object_request.type,
                spawn_object_request.id,
                spawn_object_request.attach_to,
                spawn_object_request.random_pose,
                attr_preview))

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

    def _collect_object_files(self):
        """
        Collect all object definition files from the objects_definition_file parameter.
        Supports single file or comma-separated list of files.
        Prepends objects_directory to relative paths.
        :return: list of file paths to process
        """
        files_str = (self.objects_definition_file or "").strip()

        if not files_str:
            return []

        return [
            os.path.join((self.objects_directory or "").strip(), f.strip())
            for f in files_str.split(',')
            if f.strip()
        ]

    def _auto_load_blueprints(self, blueprints_dir):
        """
        Automatically load all blueprint JSON files from the given blueprints directory.
        This allows users to only specify object files in SENSORS, while blueprints
        are loaded automatically.

        :param blueprints_dir: Path to the blueprints directory
        :return: tuple of (blueprint_files_loaded, merged_blueprints, blueprint_ids)
        """
        merged_blueprints = []
        blueprint_ids = set()
        files_loaded = []

        self.loginfo("Auto-loading blueprints from: {}".format(blueprints_dir))

        # Walk through all subdirectories and find JSON files
        for root, dirs, files in os.walk(blueprints_dir):
            for filename in sorted(files):  # Sort for deterministic loading order
                if not filename.endswith('.json'):
                    continue

                filepath = os.path.join(root, filename)
                rel_path = os.path.relpath(filepath, blueprints_dir)

                try:
                    with open(filepath) as handle:
                        json_data = json.loads(handle.read())
                except json.JSONDecodeError as e:
                    self.logwarn("Invalid JSON in blueprint file {}: {}, skipping.".format(filepath, e))
                    continue
                except IOError as e:
                    self.logwarn("Could not read blueprint file {}: {}, skipping.".format(filepath, e))
                    continue

                # Only process files that contain blueprints
                blueprints_in_file = json_data.get('blueprints', [])
                if not blueprints_in_file:
                    continue

                files_loaded.append(filepath)
                self.loginfo("Loading blueprints from: {}".format(rel_path))

                for blueprint in blueprints_in_file:
                    bp_id = blueprint.get('id')
                    if bp_id in blueprint_ids:
                        self.logwarn("Duplicate blueprint id '{}' found in {}, skipping.".format(
                            bp_id, rel_path))
                        continue
                    blueprint_ids.add(bp_id)
                    merged_blueprints.append(blueprint)

        if files_loaded:
            self.loginfo("Auto-loaded {} blueprints from {} file(s).".format(
                len(merged_blueprints), len(files_loaded)))

        return files_loaded, merged_blueprints, blueprint_ids

    def _load_and_merge_definitions(self, object_files, preloaded_blueprints=None, preloaded_blueprint_ids=None):
        """
        Load multiple JSON definition files and merge their blueprints and objects.
        :param object_files: list of file paths to load
        :param preloaded_blueprints: list of blueprints already loaded (e.g., from auto-load)
        :param preloaded_blueprint_ids: set of blueprint IDs already loaded
        :return: tuple of (merged_blueprints, merged_objects)
        """
        merged_blueprints = list(preloaded_blueprints) if preloaded_blueprints else []
        merged_objects = []
        blueprint_ids = set(preloaded_blueprint_ids) if preloaded_blueprint_ids else set()
        object_ids = set()

        for filepath in object_files:
            if not os.path.exists(filepath):
                raise RuntimeError(
                    "Could not read object definitions from {}".format(filepath))

            self.loginfo("Loading object definitions from: {}".format(filepath))

            with open(filepath) as handle:
                try:
                    json_data = json.loads(handle.read())
                except json.JSONDecodeError as e:
                    raise RuntimeError(
                        "Invalid JSON in file {}: {}".format(filepath, e))

            # Merge blueprints (check for duplicates)
            for blueprint in json_data.get('blueprints', []):
                bp_id = blueprint.get('id')
                if bp_id in blueprint_ids:
                    self.logwarn("Duplicate blueprint id '{}' found in {}, skipping.".format(
                        bp_id, filepath))
                    continue
                blueprint_ids.add(bp_id)
                merged_blueprints.append(blueprint)

            # Merge objects (check for duplicates)
            for obj in json_data.get('objects', []):
                obj_id = obj.get('id')
                if obj_id in object_ids:
                    self.logwarn("Duplicate object id '{}' found in {}, skipping.".format(
                        obj_id, filepath))
                    continue
                object_ids.add(obj_id)
                merged_objects.append(obj)

        self.loginfo("Loaded {} blueprints and {} objects from {} file(s).".format(
            len(merged_blueprints), len(merged_objects), len(object_files)))

        return merged_blueprints, merged_objects

    def spawn_objects(self):
        """
        Processes the input object definition file(s).
        Supports both single file and multiple files.
        Automatically loads blueprints from the blueprints/ subdirectory.
        """

        # Collect all definition files
        object_files = self._collect_object_files()

        if not object_files:
            raise RuntimeError(
                "No object definition files specified. Set 'objects_definition_file' parameter.")

        # Auto-load all blueprints from the blueprints/ directory
        if os.path.exists(self.blueprints_directory) and os.path.isdir(self.blueprints_directory):
            _, preloaded_blueprints, preloaded_blueprint_ids = self._auto_load_blueprints(self.blueprints_directory)
        else:
            preloaded_blueprints = []
            preloaded_blueprint_ids = set()

        # Load and merge all definition files (with preloaded blueprints)
        self.blueprints, self.objects = self._load_and_merge_definitions(
            object_files, preloaded_blueprints, preloaded_blueprint_ids)

        global_sensors = [obj for obj in self.objects if obj['type'].split('.')[0] == 'sensor']

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

            global_vehicles = [obj for obj in self.objects if obj['type'].split('.')[0] == 'vehicle' or obj['type'].split('.')[0] == 'walker']

            for vehicle in global_vehicles:
                for actor_info in actor_info_list.actors:
                    if actor_info.type == vehicle["type"] and actor_info.rolename == vehicle["id"]:
                        vehicle["carla_id"] = actor_info.id

        # iterate through all remaining top-level objects (without sensors)
        global_objects = [obj for obj in self.objects if obj['type'].split('.')[0] != 'sensor']

        for obj in global_objects:
            self.process_object(obj, None)

        self.loginfo("All objects spawned.")

    def process_vehicle(self, vehicle, parent):
        """
        Create the vehicle defined by the input dict
        :param vehicle: vehicle input dict
        :param parent: id of attached parent
        """

        if parent is not None:
            self.logerr(
                    "Could not spawn vehicle {}, because parent exists.".format(vehicle["id"]))
            return

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

            if spawn_param_used is False and "spawn_point" in vehicle:
                # get spawn point from config file
                try:
                    spawn_point = self.resolve_spawn_point(vehicle["spawn_point"])
                    self.loginfo("Spawn point from configuration file")
                except KeyError as e:
                    self.logerr("{}: Could not use the spawn point from config file, ".format(vehicle["id"]) +
                                "the mandatory attribute {} is missing, a random spawn point will be used".format(e))
                    raise
                except Exception as e:
                    self.logerr("{}: Could not resolve spawn point from config file: {}".format(vehicle["id"], e))
                    raise

            if spawn_param_used is False and "spawn_point" not in vehicle:
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
                    vehicle["name"] = vehicle["id"]

                    # recursively process sensor objects:
                    for object in vehicle.get('sensors', []):
                        self.process_object(object, vehicle)
                    # recursively process child objects:
                    for object in vehicle.get('children', []):
                        self.process_object(object, vehicle)

    def preprocess_object(self, object, parent):
        """
        Preprocess sensors and groups before actual spawning
        :param object: object input dict
        :param parent: parent object
        """
        
        # use spawn_point if object is non-pseudo top level object or contains spawn_point
        if (parent is None and "pseudo" not in object["type"]) or 'spawn_point' in object:
            object['local_transform'] = self.resolve_spawn_point(object['spawn_point'])
        # if object attached to a parent, or is a 'pseudo_actor', allow default pose
        elif 'spawn_point' not in object:
            object['local_transform'] = self.create_spawn_point(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        # set name, attached_vehicle_id, and transform by considering parent object
        if parent is not None:
            
            if parent['type'] == 'vehicle' and 'attached_vehicle_id' in parent:
                raise RuntimeError("Object {} will not be spawned, the parent vehicle {} is already attached to another vehicle.".format(object["id"], parent['id']))

            elif parent['type'].split('.')[0] == 'vehicle':
                object["name"] = parent['name'] + "/" + object["id"]
                object["transform"] = object['local_transform']
                object['attached_vehicle_id'] = parent['response_id']

            elif 'attached_vehicle_id' in parent:
                object["name"] = parent['name'] + "/" + object["id"]
                object["id"] = parent['id'] + "/" + object["id"]
                object['attached_vehicle_id'] = parent['attached_vehicle_id']
                object['transform'] = self.extend_spawn_point(parent['transform'], object['local_transform'])
        else:
            object["name"] = object["id"]
            object["transform"] = object['local_transform']
            object['attached_vehicle_id'] = 0

        # check if object name already exists
        if object["name"] in self.object_names:
            raise NameError
        self.object_names.append(object["name"])


    def process_sensor(self, sensor, parent):
        """
        Create the sensor defined by the input dict
        :param sensor: sensor input dict
        :param parent: parent object
        """
        if not roscomp.ok():
            return
        
        # check if parent is a sensor
        if parent is not None and parent['type'].split('.')[0] == 'sensor':
            self.logerr(
                    "Could not spawn sensor {}, because the parent is already a sensor.".format(sensor["id"]))
            return

        try:
            # preprocess object
            self.preprocess_object(sensor, parent)

            # spawn the sensor object
            spawn_object_request = roscomp.get_service_request(SpawnObject)
            spawn_object_request.type = sensor["type"]
            spawn_object_request.id = sensor["id"]
            spawn_object_request.attach_to = sensor['attached_vehicle_id']
            spawn_object_request.transform = sensor['transform']
            spawn_object_request.random_pose = False  # never set a random pose for a sensor

            attached_objects = []
            for attribute, value in sensor.items():
                # skip general attributes
                if attribute in ["id", "type", "name", "spawn_point", "local_transform", "transform", "attached_vehicle_id", "response_id"]:
                    continue
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

            # keep track of sensors for deconstruction
            if parent is None:
                self.global_sensors.append(sensor['response_id'])
            else:
                self.vehicles_sensors.append(sensor['response_id'])

        except NameError as e:
            self.logerr("Sensor name '{}' is only allowed to be used once. The second one will be ignored: {}".format(
                sensor["id"], e))
            return

        except RuntimeError as e:
            self.logerr(
                "Sensor {} will not be spawned: {}".format(sensor["id"], e))
            return

        except:
            self.logerr(
                "Sensor {} will not be spawned".format(sensor["id"]))
            return

    def process_group(self, group, parent):
        """
        Create the group defined by the input dict
        :param group: group input dict
        :param parent: parent object
        """
        if not roscomp.ok():
            return

        # check if parent is a sensor
        if parent is not None and parent['type'].split('.')[0] == 'sensor':
            self.logerr(
                    "Could not spawn group {}, because the parent is a sensor.".format(group["id"]))
            return

        try:
            
            # preprocess object
            self.preprocess_object(group, parent)            

            # spawn a potential physical object
            if 'physical_object' in group:
                spawn_object_request = roscomp.get_service_request(SpawnObject)
                spawn_object_request.type = group["physical_object"]
                spawn_object_request.id = group["id"]
                spawn_object_request.attach_to = group['attached_vehicle_id']
                spawn_object_request.transform = group['transform']
                spawn_object_request.random_pose = False # never set a random pose for an object

                group_spawned = False
                while not group_spawned and roscomp.ok():

                    group['response_id'] = self.spawn_object(spawn_object_request)
                    if group['response_id'] != -1:
                        group_spawned = True
                        self.players.append(group['response_id'])
            
            # spawn the child objects
            for child in group['children']:
                self.process_object(child, group)

        except NameError as e:
            self.logerr("Group name '{}' is only allowed to be used once. The second one will be ignored: {}".format(
                group["id"], e))
            return

        except Exception as e:
            self.logerr(
                "Group {} will not be spawned: {}".format(group["id"], e))
            return

        # broadcast static static transform from parent to group
        static_transform = geometry_msgs.msg.TransformStamped()
        if ROS_VERSION == 1:
            broadcaster = tf2_ros.StaticTransformBroadcaster()
            static_transform.header.stamp = rospy.Time.now()
        elif ROS_VERSION == 2:
            broadcaster = tf2_ros.StaticTransformBroadcaster(self)
            static_transform.header.stamp = self.get_clock().now().to_msg()

        if parent is None:
            static_transform.header.frame_id = self.world_frame
        else:
            static_transform.header.frame_id = parent.get('name', '')

        static_transform.child_frame_id = group.get("name", "")
        if not static_transform.header.frame_id or not static_transform.child_frame_id:
            self.logwarn(
                "Skipping invalid static transform for group '{}': frame_id='{}', child_frame_id='{}'".format(
                    group.get("id", "<unknown>"),
                    static_transform.header.frame_id,
                    static_transform.child_frame_id))
            return

        static_transform.transform.translation.x = group['local_transform'].position.x
        static_transform.transform.translation.y = group['local_transform'].position.y
        static_transform.transform.translation.z = group['local_transform'].position.z
        static_transform.transform.rotation = group['local_transform'].orientation

        broadcaster.sendTransform(static_transform) 

    def process_blueprint(self, object, parent):
        """
        Extend existing blueprint with object information
        :param group: object input dict
        :param parent: parent object
        """

        # take blueprint and add object information
        blueprint_found = False

        for blueprint in self.blueprints:
            if blueprint['id'] != object['type'].split('.')[1]: continue
            
            # check if blueprint is of type blueprint
            if blueprint['type'].split('.')[0] == 'blueprint':
                self.logerr(
                        "Could not use blueprint {}, because the type is already a blueprint.".format(blueprint["id"]))
                return

            blueprint_found = True
            extended_object = copy.deepcopy(blueprint)
            extended_object["id"] = object["id"]

            if "spawn_point" in object:
                extended_object["spawn_point"] = object["spawn_point"]

            self.process_object(extended_object, parent)

        # check if blueprint was found
        if not blueprint_found:
            self.logerr("Blueprint {} not found.".format(object["type"].split('.')[1]))

    def process_object(self, obj, parent):
        """
        General function to process an object of any type
        :param obj: object input dict
        :param parent: parent object
        """

        # get the corresponding function and call it
        func = self.object_type_map.get(obj["type"].split('.')[0], None)
        if func:
            func(obj, parent)
        else:
            self.logwarn(
                    "Object with type {} is not a vehicle, a walker, a sensor, a group, or a blueprint, ignoring".format(obj["type"]))

    def create_spawn_point(self, x, y, z, roll, pitch, yaw):
        """
        Create a spawn point from the input parameters
        """

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

    def extend_spawn_point(self, base, shift):
        """
        Extend a spawn point by another spawn point.
        The shift position is rotated by the base orientation before being added.
        Uses 'sxyz' rotation order (same as transforms3d default).
        param base: base spawn point (parent)
        param shift: shift spawn point (child, relative to parent)
        """

        spawn_point = Pose()

        # transform base orientation to euler angles
        base_orientation = list(quat2euler([base.orientation.w,
                                base.orientation.x,
                                base.orientation.y,
                                base.orientation.z]))

        base_roll = base_orientation[0]
        base_pitch = base_orientation[1]
        base_yaw = base_orientation[2]

        # Rotate shift position by base orientation using 'sxyz' order
        # Order: first Roll (X), then Pitch (Y), then Yaw (Z)
        
        # Roll (X-axis) first
        cos_roll = math.cos(base_roll)
        sin_roll = math.sin(base_roll)
        x1 = shift.position.x
        y1 = shift.position.y * cos_roll - shift.position.z * sin_roll
        z1 = shift.position.y * sin_roll + shift.position.z * cos_roll

        # Pitch (Y-axis)
        cos_pitch = math.cos(base_pitch)
        sin_pitch = math.sin(base_pitch)
        x2 = x1 * cos_pitch + z1 * sin_pitch
        y2 = y1
        z2 = -x1 * sin_pitch + z1 * cos_pitch

        # Yaw (Z-axis) last
        cos_yaw = math.cos(base_yaw)
        sin_yaw = math.sin(base_yaw)
        rotated_x = x2 * cos_yaw - y2 * sin_yaw
        rotated_y = x2 * sin_yaw + y2 * cos_yaw
        rotated_z = z2

        # Add rotated position to base position
        spawn_point.position.x = base.position.x + rotated_x
        spawn_point.position.y = base.position.y + rotated_y
        spawn_point.position.z = base.position.z + rotated_z

        shift_orientation = list(quat2euler([shift.orientation.w,
                                shift.orientation.x,
                                shift.orientation.y,
                                shift.orientation.z]))

        # add orientation in euler angles
        spawn_point_orientation = [0, 0, 0]
        spawn_point_orientation[0] = base_orientation[0] + shift_orientation[0]
        spawn_point_orientation[1] = base_orientation[1] + shift_orientation[1]
        spawn_point_orientation[2] = base_orientation[2] + shift_orientation[2]


        # transform orientation to quaternion
        quat = euler2quat(spawn_point_orientation[0], spawn_point_orientation[1], spawn_point_orientation[2])

        # set orientation
        spawn_point.orientation.w = quat[0]
        spawn_point.orientation.x = quat[1]
        spawn_point.orientation.y = quat[2]
        spawn_point.orientation.z = quat[3]

        return spawn_point

    def check_spawn_point_param(self, spawn_point_parameter):
        components = spawn_point_parameter.split(',')
        num_components = len(components)

        if num_components == 6:
            x, y, z, roll, pitch, yaw = map(float, components)
            return self.create_spawn_point(x, y, z, roll, pitch, yaw)

        elif num_components == 7 and components[-1] == 'wgs84':
            lat, lon, alt, roll, pitch, yaw = map(float, components[:6])
            return self.wgs84_to_carla_spawn_point(lat, lon, alt, roll, pitch, yaw)

        elif num_components == 4:
            x, y, z, yaw = map(float, components)
            return self.create_spawn_point(x, y, z, 0.0, 0.0, yaw)

        elif num_components == 5 and components[-1] == 'wgs84':
            lat, lon, alt, yaw = map(float, components[:4])
            return self.wgs84_to_carla_spawn_point(lat, lon, alt, 0.0, 0.0, yaw)

        self.logwarn("Invalid spawnpoint '{}'".format(spawn_point_parameter))
        return None

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
