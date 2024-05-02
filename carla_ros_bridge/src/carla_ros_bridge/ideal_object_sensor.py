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

import carla_common.transforms as trans
import ros_compatibility as roscomp

import tf2_ros

from tf2_geometry_msgs import do_transform_pose

import math

from rclpy.time import Time
from rclpy.duration import Duration

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
        
        # Set up Buffer and TransformListener to lookup transforms between frames
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, node, spin_thread=False)

        # Set up TransformBroadcaster to publish sensor transform
        if ROS_VERSION == 1:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        elif ROS_VERSION == 2:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(node)
        
        # Extract (relative) spawn pose
        self.relative_spawn_pose = relative_spawn_pose

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
    
    def check_visibility(self, target_pose_in_sensor_frame, timestamp, id): 
        # Calculate azimut, elevation and distance between sensor and target
        azimut_deg, elevation_deg, distance = self.calculate_azimut_elevation_distance(target_pose_in_sensor_frame, timestamp)

        # Check if target is inside the range and fov of the sensor
        if distance > self.range:
            return False
        elif azimut_deg < self.left_fov:
            return False
        elif azimut_deg > self.right_fov:
            return False
        elif elevation_deg > self.upper_fov:
            return False
        elif elevation_deg < self.lower_fov:
            return False
        else:
            return True
    

    def calculate_azimut_elevation_distance(self, target_pose_in_sensor_frame, timestamp):
        # Get location vaules of pose
        dx = target_pose_in_sensor_frame.position.x
        dy = target_pose_in_sensor_frame.position.y
        dz = target_pose_in_sensor_frame.position.z

        # Calculate distance between target and sensor
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        # Calculate azimuth between target and sensor based on sensor KOS
        azimut_rad = math.atan2(dy, dx)
        azimut_deg = math.degrees(azimut_rad)

        # Calculate elevation between target and sensor based on sensor KOS
        elevation_rad = math.asin(dz/distance)
        elevation_deg = math.degrees(elevation_rad)

        return azimut_deg, elevation_deg, distance
    

    def get_ros_transform(self, timestamp):
        # Get transform of idealObjectSensor
        if not self.relative_spawn_pose:
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

        transform.transform.translation.x = self.relative_spawn_pose.position.x
        transform.transform.translation.y = self.relative_spawn_pose.position.y
        transform.transform.translation.z = self.relative_spawn_pose.position.z

        transform.transform.rotation.x = self.relative_spawn_pose.orientation.x
        transform.transform.rotation.y = self.relative_spawn_pose.orientation.y
        transform.transform.rotation.z = self.relative_spawn_pose.orientation.z
        transform.transform.rotation.w = self.relative_spawn_pose.orientation.w

        return transform
    

    def publish_tf(self, timestamp):
        # Publish transform of idealObjectSensor
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
        # Publish transform of idealObjectSensor at timestamp
        self.publish_tf(timestamp)

        # Generate object array to publish sensor data
        ros_objects = ObjectArray()
        ros_objects.header = self.get_msg_header(frame_id="carla_map", timestamp=timestamp)

        # Get transform from idealIbjectSensor in carla_map
        sensor_frame = self.get_prefix()
        time_latest_tf = Time(seconds=0)
        duration_timeout = Duration(seconds=0)
        try:
            tf_carla_map_to_sensor = self.tf_buffer.lookup_transform(sensor_frame, 'carla_map', time_latest_tf, duration_timeout)
        except:
            self.node.loginfo("{}: Could not transform {} to {} at the Frame {}".format(
                self.__class__.__name__, sensor_frame, 'carla_map', frame))
            return

        # Iterate over all dynamic actors
        for actor_id in self.actor_list.keys():
            
            # Currently only vehicles and walkers are added to the object array
            if self.parent is None or self.parent.uid != actor_id:
                actor = self.actor_list[actor_id]
                if isinstance(actor, Vehicle) or isinstance(actor, Walker):

                    # Get target pose in carla_map and transform to sensor frame
                    target_pose_in_carla_map = actor.get_current_ros_pose()
                    target_pose_in_sensor_frame = do_transform_pose(target_pose_in_carla_map, tf_carla_map_to_sensor)

                    # Check visibility of the target
                    if self.check_visibility(target_pose_in_sensor_frame, timestamp, actor_id):
                        ros_objects.objects.append(actor.get_object_info())

        # Iterate over all static vehicles
        if(self.node.parameters['publish_static_vehicles']):
            for object_key, object_value in self.OBJECT_LABELS.items():
                static_vehicles = self.world.get_environment_objects(object_key)

                for vehicle in static_vehicles:
                    # Take only vehicles with bounding_box attribute set
                    if hasattr(vehicle, "bounding_box"):

                        # Get target pose in carla_map and transform to sensor frame
                        target_pose_in_carla_map = trans.carla_transform_to_ros_pose(vehicle.transform)
                        target_pose_in_sensor_frame = do_transform_pose(target_pose_in_carla_map, tf_carla_map_to_sensor)

                        # Check visibility of the target
                        if self.check_visibility(target_pose_in_sensor_frame, timestamp, vehicle.id):
                            vehicle_obj = self._get_vehicle_from_environment_objects(vehicle, object_value)
                            ros_objects.objects.append(vehicle_obj)

        self.object_publisher.publish(ros_objects)