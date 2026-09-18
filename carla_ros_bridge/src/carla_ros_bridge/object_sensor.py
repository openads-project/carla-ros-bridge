#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a object sensor
"""

import carla
import carla_common.transforms as trans
import ctypes
import re

from carla_ros_bridge.pseudo_actor import PseudoActor
from carla_ros_bridge.vehicle import Vehicle
from carla_ros_bridge.walker import Walker

from derived_object_msgs.msg import ObjectArray, Object
from shape_msgs.msg import SolidPrimitive
from transforms3d.euler import euler2quat

class ObjectSensor(PseudoActor):

    """
    Pseudo object sensor
    """

    OBJECT_LABELS = {
        carla.CityObjectLabel.Car: Object.CLASSIFICATION_CAR,
        carla.CityObjectLabel.Truck: Object.CLASSIFICATION_TRUCK,
        carla.CityObjectLabel.Bus: Object.CLASSIFICATION_OTHER_VEHICLE,
        carla.CityObjectLabel.Motorcycle:Object.CLASSIFICATION_MOTORCYCLE,
        carla.CityObjectLabel.Bicycle: Object.CLASSIFICATION_BIKE,
        carla.CityObjectLabel.Pedestrians: Object.CLASSIFICATION_PEDESTRIAN
    }

    # Strip off Unreal parked-vehicle bps, which are into several separately tagged
    # static meshes ("BP_EuropeanHGV_Parked_C_10_SM_0", "..._SM_1")
    ENVIRONMENT_OBJECT_MESH_SUFFIX = re.compile(r"_SM_\d+$")

    def __init__(self, uid, name, parent, node, actor_list, world):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: CompatibleNode
        :param actor_list: current list of actors
        :type actor_list: map(carla-actor-id -> python-actor-object)
        """

        super(ObjectSensor, self).__init__(uid=uid,
                                           name=name,
                                           parent=parent,
                                           node=node)
        self.actor_list = actor_list
        self.world = world
        self.node = node
        self.object_publisher = node.new_publisher(ObjectArray,
                                                   self.get_topic_prefix(),
                                                   qos_profile=10)

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        super(ObjectSensor, self).destroy()
        self.actor_list = None
        self.node.destroy_publisher(self.object_publisher)

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.objects"
    
    def _get_vehicle_from_environment_objects(self, environment_object, object_classification):
        obj = Object(header=self.get_msg_header("carla_map"))
        obj.id = ctypes.c_uint32(environment_object.id).value
        obj.pose = trans.carla_transform_to_ros_pose(
            self._get_environment_object_transform(environment_object))

        if self.node.parameters['ignore_altitude']:
            obj.pose.position.z = 0.0

        if self.node.parameters['ignore_tilt']:
            obj.pose.orientation = trans.reset_tilt_angles(
                obj.pose.orientation)

        # only static obj
        obj.twist = trans.carla_velocity_to_ros_twist(carla.Vector3D(0.0, 0.0, 0.0), carla.Vector3D(0.0, 0.0, 0.0))
        obj.accel = trans.carla_acceleration_to_ros_accel(carla.Vector3D(0.0, 0.0, 0.0))
        obj.shape.type = SolidPrimitive.BOX
        obj.shape.dimensions.extend([
            environment_object.bounding_box.extent.x * 2.0,
            environment_object.bounding_box.extent.y * 2.0,
            environment_object.bounding_box.extent.z * 2.0])
        obj.classification = object_classification
        obj.classification_certainty = 1
        obj.object_classified = True

        return obj

    def get_static_vehicles(self):
        """Gets every parked map vehicle once, as (environment_object, classification).

        CARLA reports a parked vehicle as one static mesh per blueprint
        component, each carrying its own semantic tag and bounding box, so
        labelling all of them puts multiple box onto one real vehicle.
        Here the largest volume box wins, others are discarded.
        """
        if getattr(self, "_static_vehicles", None) is not None:
            return self._static_vehicles

        largest_per_vehicle = {}
        for object_label, classification in self.OBJECT_LABELS.items():
            for environment_object in self.world.get_environment_objects(object_label):
                # Only vehicles with a bounding box can be turned into an Object.
                if not hasattr(environment_object, "bounding_box"):
                    continue
                vehicle = self.ENVIRONMENT_OBJECT_MESH_SUFFIX.sub("", environment_object.name)
                extent = environment_object.bounding_box.extent
                volume = extent.x * extent.y * extent.z
                previous = largest_per_vehicle.get(vehicle)
                if previous is None or volume > previous[0]:
                    largest_per_vehicle[vehicle] = (volume, environment_object, classification)

        self._static_vehicles = [
            (environment_object, classification)
            for _, environment_object, classification in largest_per_vehicle.values()
        ]
        return self._static_vehicles

    def _get_environment_object_transform(self, environment_object):
        box = environment_object.bounding_box
        return carla.Transform(box.location, box.rotation)

    def _get_environment_object_world_vertices(self, environment_object):
        return environment_object.bounding_box.get_world_vertices(carla.Transform())

    def _get_static_vehicles(self, ros_objects):
        # iterate over all static vehicles, one box per vehicle
        for vehicle, object_value in self.get_static_vehicles():
            vehicle_obj = self._get_vehicle_from_environment_objects(vehicle, object_value)
            ros_objects.objects.append(vehicle_obj)

        return ros_objects

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        On update carla_map sends:
        - tf global frame
        :return:
        """
        ros_objects = ObjectArray()
        ros_objects.header = self.get_msg_header(frame_id="carla_map", timestamp=timestamp)
        for actor_id in self.actor_list.keys():
            # currently only Vehicles and Walkers are added to the object array
            if self.parent is None or self.parent.uid != actor_id:
                actor = self.actor_list[actor_id]
                if isinstance(actor, Vehicle):
                    ros_objects.objects.append(actor.get_object_info())
                elif isinstance(actor, Walker):
                    ros_objects.objects.append(actor.get_object_info())
        
        if(self.node.parameters['publish_static_vehicles']):
            # add also static vehicles to ros_objects.object array
            ros_objects = self._get_static_vehicles(ros_objects)

        self.object_publisher.publish(ros_objects)
