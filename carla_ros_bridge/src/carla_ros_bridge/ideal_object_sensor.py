#!/usr/bin/env python
#
# Copyright (c) 2024 Institute for Automotive Engineering (ika) RWTH Aachen University
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Handle an IdealObjectSensor
"""

from carla_ros_bridge.vehicle import Vehicle
from carla_ros_bridge.walker import Walker

from derived_object_msgs.msg import ObjectArray

from carla_ros_bridge.object_sensor import ObjectSensor

from carla.libcarla import Location

try:
    import queue
except ImportError:
    import Queue as queue

import carla_common.transforms as trans
import ros_compatibility as roscomp

import tf2_ros

import math

ROS_VERSION = roscomp.get_ros_version()

class IdealObjectSensor(ObjectSensor):

    """
    IdealObjectSensor
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, actor_list, world, attributes):
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
        :param world: current carla world object
        :type world: carla.World
        :param attributes: attributes of IdealObjectSensor
        :type attributes: diagnostic_msgs/KeyValue[]
        """
        
        super(IdealObjectSensor, self).__init__(uid=uid,
                                                      name=name,
                                                      parent=parent,
                                                      node=node,
                                                      actor_list=actor_list, 
                                                      world=world)
        self.node = node
        self.object_publisher = node.new_publisher(ObjectArray,
                                                   self.get_topic_prefix(),
                                                   qos_profile=10)
        
        self.queue = queue.Queue()
        if ROS_VERSION == 1:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        elif ROS_VERSION == 2:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(node)
        
        # Extract spawn pose and convert (relative) position to carla.Location
        self.position = Location(x=relative_spawn_pose.position.x, y=relative_spawn_pose.position.y, z=relative_spawn_pose.position.z)
        self.orientation = relative_spawn_pose.orientation

        # Extract relevant attributes and convert to float
        for attribute in attributes:
            if attribute.key == "range":
                self.range = float(attribute.value)
            elif attribute.key == "left_fov":
                self.left_fov = float(attribute.value)
            elif attribute.key == "right_fov":
                self.right_fov = float(attribute.value)
            elif attribute.key == "upper_fov":
                self.upper_fov = float(attribute.value)
            elif attribute.key == "lower_fov":
                self.lower_fov = float(attribute.value)

        # Check relevant attributes and set default values if not available
        try: 
            self.range
        except:
            self.range = 15.0
            self.node.logwarn(
                "No range attribute found for IdealObjectSensor. Using default value of {} meters.".format(self.range)
            )

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        super(IdealObjectSensor, self).destroy()
        self.actor_list = None
        self.node.destroy_publisher(self.object_publisher)

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.ideal_objects"
    
    def check_visibility(self, sensor_location, target_location, id): 

        # Calculate the Euclidean distance between sensor and target 
        distance = sensor_location.distance(target_location)

        # Calculate azimuth and elevation between sensor and target
        azimuth_deg, elevation_deg = self.calculate_azimuth_and_elevation(sensor_location, target_location, distance, id)

        # Check if the target is inside the range and fov of the sensor
        if distance > self.range:
            return False
        elif azimuth_deg < self.left_fov:
            return False
        elif azimuth_deg > self.right_fov:
            return False
        elif elevation_deg > self.upper_fov:
            return False
        elif elevation_deg < self.lower_fov:
            return False
        else:
            return True
    
    def calculate_azimuth_and_elevation(self, sensor_location, target_location, distance, id):

        # Calculate the vector from source to target
        dx = target_location.x - sensor_location.x
        dy = target_location.y - sensor_location.y
        dz = target_location.z - sensor_location.z

        # Calculate azimuth in degrees
        azimuth_rad = math.atan(dx/dz)
        azimuth_deg = math.degrees(azimuth_rad)

        # Calculate elevation in degrees
        elevation_rad = math.asin(dy/distance)
        elevation_deg = math.degrees(elevation_rad)

        if distance < 50:
            print(f"Distance: {round(distance, 2)}, ID: {id}, dx: {round(dx,2)}, dy: {round(dy, 2)}, dz: {round(dz, 2)}, Azimuth: {round(azimuth_deg, 2)}, Elevation: {round(elevation_deg, 2)}")

        return azimuth_deg, elevation_deg
    
    def get_ros_transform(self, timestamp):
        if not self.position and not self.orientation:
            self.node.logwarn("{}: No relative spawn pose defined.".format(self.get_prefix()))
            return
        if self.parent is not None:
            frame_id = self.parent.get_prefix()
        else:
            frame_id = "carla_map"
        child_frame_id = self.get_prefix()
        
        transform = tf2_ros.TransformStamped()
        transform.header.stamp = roscomp.ros_timestamp(sec=timestamp + self.node.parameters["start_unix_time_stamp"], from_sec=True)
        transform.header.frame_id = frame_id
        transform.child_frame_id = child_frame_id

        transform.transform.translation.x = self.position.x
        transform.transform.translation.y = self.position.y
        transform.transform.translation.z = self.position.z

        transform.transform.rotation.x = self.orientation.x
        transform.transform.rotation.y = self.orientation.y
        transform.transform.rotation.z = self.orientation.z
        transform.transform.rotation.w = self.orientation.w

        return transform


    def publish_tf(self, timestamp):
        transform =self.get_ros_transform(timestamp)
        try:
            self._tf_broadcaster.sendTransform(transform)
        except roscomp.exceptions.ROSException:
            if roscomp.ok():
                self.node.logwarn("Sensor {} failed to send transform.".fromat(self.uid))

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        On update carla_map sends:
        - tf global frame
        :return:
        """

        try:
            self.publish_tf(timestamp)
        except queue.Empty:
            return
        
        ros_objects = ObjectArray()
        ros_objects.header = self.get_msg_header(frame_id="carla_map", timestamp=timestamp)

        # Construct sensor location
        if not self.parent:
            # Location for idealObjectSensor without vehicle parent
            sensor_location = self.position
        else:
            """       
                - Get the vehicle that the IdealObjectSensor is appended
                - This can be either ego-vehicle or hero-vehicle based on the sensors.json definitions
                - Calculate position of the IdealObjectSensor located in vehicle
            """
            ego_vehicle = self.actor_list[self.parent.uid]  
            ego_vehicle_location = ego_vehicle.carla_actor.get_location()

            # Location for idealObjectSensor with vehicle parent
            sensor_location = self.position + ego_vehicle_location

        # Iterate over all dynamic actors
        for actor_id in self.actor_list.keys():
            
            # Currently only vehicles and walkers are added to the object array
            if self.parent is None or self.parent.uid != actor_id:
                actor = self.actor_list[actor_id]
                if isinstance(actor, Vehicle) or isinstance(actor, Walker):
                    
                    # Get the location of the target
                    target_location = actor.carla_actor.get_location()

                    # Check visibility of the target
                    if self.check_visibility(sensor_location, target_location, actor.carla_actor.id):
                        ros_objects.objects.append(actor.get_object_info())

        # Iterate over all static vehicles
        if(self.node.parameters['publish_static_vehicles']):
            for object_key, object_value in self.OBJECT_LABELS.items():
                static_vehicles = self.world.get_environment_objects(object_key)

                for vehicle in static_vehicles:
                    # Take only vehicles with bounding_box attribute set
                    if hasattr(vehicle, "bounding_box"):
                        
                        # Get the location of the target
                        target_location = vehicle.transform.location

                        # Check visibility of the target
                        if self.check_visibility(sensor_location, target_location, vehicle.id):
                            vehicle_obj = self._get_vehicle_from_environment_objects(vehicle, object_value)
                            ros_objects.objects.append(vehicle_obj)

        self.object_publisher.publish(ros_objects)