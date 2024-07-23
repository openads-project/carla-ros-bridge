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
import math

import carla
import carla_common.transforms as trans
from carla_ros_bridge.vehicle import Vehicle
from carla_ros_bridge.walker import Walker
from carla_ros_bridge.object_sensor import ObjectSensor

from derived_object_msgs.msg import ObjectArray
from geometry_msgs.msg import Point, PointStamped

from rclpy.time import Time
from rclpy.duration import Duration
import ros_compatibility as roscomp

import tf2_ros
from tf2_geometry_msgs import do_transform_point

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
        :param name: name identifying this object
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

        # Set default values, boundaries and unit for sensor parameters so that they are available when needed
        attributes_dict = {
            "range":                {"default": 15.0,   "lower_boundary": 0},
            "left_fov":             {"default": -180.0, "lower_boundary": -180, "upper_boundary": 0},
            "right_fov":            {"default": 180.0,  "lower_boundary": 0,    "upper_boundary": 180},
            "upper_fov":            {"default": 90.0,  "lower_boundary": 0,    "upper_boundary": 90},
            "lower_fov":            {"default": -90.0, "lower_boundary": -90, "upper_boundary": 0},
            "min_corner_amount":    {"default": 3,      "lower_boundary": 1,    "upper_boundary": 8},
            "distance_tolerance":   {"default": 10.0,   "lower_boundary": 0} # 10 Meters based on the length of a truck
        }

        # Extract and check attributes and set default values if not available or values are not set in parameter boundaries
        for key, current_dict in attributes_dict.items():
            try:
                # Extract relevant attributes and convert to float
                attribute = next((attribute for attribute in attributes if attribute.key == key), None)
                setattr(self, key, float(attribute.value))
                # Boundary check
                if getattr(self, key) < current_dict.get("lower_boundary") or ("upper_boundary" in current_dict.keys() and getattr(self, key) > current_dict.get("upper_boundary")):
                    setattr(self, key, getattr(current_dict, "default"))
                    self.node.logwarn(
                        "{} attribute for IdealObjectSensor is not in parameter boundaries! Using default value of {}.".format(
                            key, current_dict.get("default")))
            except:
                # Attribute not available
                setattr(self, key, current_dict.get("default"))
                self.node.logwarn(
                    "No {} attribute found for IdealobjectSensor. Using default value of {}.".format(
                        key, current_dict.get("default")))

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
    
    def calculate_azimuth_elevation(self, target_point_in_sensor_frame, distance):
        # Get location vaules of pose
        dx = target_point_in_sensor_frame.x
        dy = target_point_in_sensor_frame.y
        dz = target_point_in_sensor_frame.z

        # Calculate azimuth and elevation between target and sensor based on sensor frame
        azimuth = math.degrees(math.atan2(dy, dx))
        elevation = math.degrees(math.asin(dz/distance))

        return azimuth, elevation
    
    def check_visibility(self, carla_location_target_in_carla_map, ros_corners_in_sensor_frame, carla_corners_in_carla_map, carla_location_sensor_in_carla_map):

        # FILTER 1
        # Calculate distance between sensor and target
        distance = carla_location_sensor_in_carla_map.distance(carla_location_target_in_carla_map)

        # Filter objects that are far outside the sensor range (including distance tolerance)
        if abs(distance-self.distance_tolerance) > self.range:
            return False
        
        # FILTER 2
        # Filter corners that are outside the sensor range and return if not enough corners are visible
        corner_list_filter_2 = list()

        for corner_num, corner in enumerate(carla_corners_in_carla_map):
            corner_distance = carla_location_sensor_in_carla_map.distance(corner)
            if corner_distance > self.range:
                continue
            corner_list_filter_2.append([corner_num, corner, corner_distance])
        
        if len(corner_list_filter_2) < self.min_corner_amount:
            return False
        
        # FILTER 3
        # Filter corners outside the sensor FOV and return if not enough corners are visible
        corner_list_filter_3 = list()

        for corner in corner_list_filter_2:
            # Get correct Position of corner in sensor frame (ROS environment)
            ros_corner_pointstamped = ros_corners_in_sensor_frame[corner[0]]

            # Calculate azimuth and elevation
            azimuth, elevation = self.calculate_azimuth_elevation(ros_corner_pointstamped.point, corner[2])
            
            # Check if corner is inside the sensor FOV
            if azimuth < self.left_fov: continue
            if azimuth > self.right_fov: continue
            if elevation > self.upper_fov: continue
            if elevation < self.lower_fov: continue

            corner_list_filter_3.append(corner[1])
        
        if len(corner_list_filter_3) < self.min_corner_amount:
            return False
        
        # FILTER 4
        # Filter corners that are covered by other objects and return if not enough corners are visible
        corner_list_filter_4 = list()

        for corner in corner_list_filter_3:
            hit = False
            # Send ray from sensor to corner and check for objects
            hit_points = self.world.cast_ray(carla_location_sensor_in_carla_map, corner)
            if hit_points:
                for hit_point in hit_points:
                    # Skip hit points with the label "Roads" located directly next to bounding box of target
                    if hit_point.label is carla.CityObjectLabel.Roads and hit_point.location.distance(corner) > 0.15:
                        continue
                    # Skip hit points with the label "NONE"
                    if hit_point.label is carla.CityObjectLabel.NONE:
                        continue
                    # Skip hit points with the label "Car" located directly next to the sensor
                    if hit_point.label is carla.CityObjectLabel.Car and hit_point.location.distance(carla_location_sensor_in_carla_map) < 2.0:
                        continue
                    # All other hits are relevant --> current corner is not visible, continue with next corner
                    hit = True
                    break
                if hit: continue
            corner_list_filter_4.append(corner)
        
        if len(corner_list_filter_4) < self.min_corner_amount:
            return False
        
        return True

    def point_to_pointstamped(self, point):
        # Convert ROS geometry_msgs/Point to ROS geometry_msgs/PointStamped
        point_stamped = PointStamped()
        point_stamped.point = point

        return point_stamped

    def convert_target_corner(self, carla_corners_in_carla_map, ros_tf_sensor_to_carla_map):

        # Convert target corners from CARLA.Location to ROS geometry_msgs/PointStamped
        ros_corners_in_carla_map_point = [trans.carla_location_to_ros_point(corner) for corner in carla_corners_in_carla_map]
        ros_corners_in_carla_map_pointstamped = [self.point_to_pointstamped(corner) for corner in ros_corners_in_carla_map_point]

        # Transform target corners from carla_map frame to sensor frame
        ros_corners_in_sensor_frame = [do_transform_point(corner, ros_tf_sensor_to_carla_map) for corner in ros_corners_in_carla_map_pointstamped]

        return ros_corners_in_sensor_frame

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
        transform = self.get_ros_transform(timestamp)
        try:
            self._tf_broadcaster.sendTransform(transform)
        except roscomp.exceptions.ROSException:
            if roscomp.ok():
                self.node.logwarn("Sensor {} failed to send transform.".format(self.uid))
                
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

        # Get ROS transform from idealIbjectSensor to carla_map and vice versa
        sensor_frame = self.get_prefix()
        time_latest_tf = Time(seconds=0)
        duration_timeout = Duration(seconds=0)
        try:
            ros_tf_carla_map_to_sensor = self.tf_buffer.lookup_transform(sensor_frame, 'carla_map' , time_latest_tf, duration_timeout)
            ros_tf_sensor_to_carla_map = self.tf_buffer.lookup_transform('carla_map', sensor_frame, time_latest_tf, duration_timeout)
        except:
            self.node.loginfo("{}: Could not transform {} to {} at the Frame {}".format(
                self.__class__.__name__, sensor_frame, 'carla_map', frame))
            return

        # Extract sensor location in carla_map from ROS transform and convert into geometry_msgs/Point
        ros_point_sensor_in_carla_map = Point(
            x=ros_tf_sensor_to_carla_map.transform.translation.x,
            y=ros_tf_sensor_to_carla_map.transform.translation.y,
            z=ros_tf_sensor_to_carla_map.transform.translation.z
        )
        carla_location_sensor_in_carla_map = trans.ros_point_to_carla_location(ros_point_sensor_in_carla_map)

        # Iterate over all dynamic actors
        for actor_id in self.actor_list.keys():

            # Currently only vehicles and walkers are added to the object array
            if self.parent is None or self.parent.uid != actor_id:
                actor = self.actor_list[actor_id]
                if isinstance(actor, Vehicle) or isinstance(actor, Walker):

                    # Get CARLA target location in carla_map
                    carla_location_target_in_carla_map = actor.carla_actor.get_location()

                    # Get corners from target BoundingBox and convert location from CARLA carla_map into ROS sensor frame
                    carla_tf_carla_map_to_target = actor.carla_actor.get_transform()
                    bounding_box = actor.carla_actor.bounding_box
                    carla_corners_in_carla_map = bounding_box.get_world_vertices(carla_tf_carla_map_to_target)
                    ros_corners_in_sensor_frame = self.convert_target_corner(carla_corners_in_carla_map, ros_tf_carla_map_to_sensor)

                    # Check visibility of the target
                    if self.check_visibility(carla_location_target_in_carla_map, ros_corners_in_sensor_frame, carla_corners_in_carla_map, carla_location_sensor_in_carla_map):
                        ros_objects.objects.append(actor.get_object_info())

        # Iterate over all static vehicles
        if(self.node.parameters['publish_static_vehicles']):
            for object_key, object_value in self.OBJECT_LABELS.items():

                static_vehicles = self.world.get_environment_objects(object_key)

                for vehicle in static_vehicles:
                    # Take only vehicles with bounding_box attribute set
                    if hasattr(vehicle, "bounding_box"):

                        # Get target location in carla_map
                        carla_location_target_in_carla_map = vehicle.transform.location

                        # Get corners from target BoundingBox and convert location from CARLA carla_map to ROS sensor frame
                        bounding_box = vehicle.bounding_box
                        carla_corners_in_carla_map = bounding_box.get_local_vertices()
                        ros_corners_in_sensor_frame = self.convert_target_corner(carla_corners_in_carla_map, ros_tf_carla_map_to_sensor)

                        # Check visibility of the target
                        if self.check_visibility(carla_location_target_in_carla_map, ros_corners_in_sensor_frame, carla_corners_in_carla_map, carla_location_sensor_in_carla_map):
                            vehicle_obj = self._get_vehicle_from_environment_objects(vehicle, object_value)
                            ros_objects.objects.append(vehicle_obj)

        self.object_publisher.publish(ros_objects)