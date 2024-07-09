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

from tf2_geometry_msgs import do_transform_point

import math

from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import Point, PointStamped

import carla

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
            elif attribute.key == 'min_corner_amount':
                self.min_corner_amount = float(attribute.value)
            elif attribute.key == "distance_variance":
                self.distance_variance = float(attribute.value)

        # Set default values, so that they are available when needed
        default_range = 15.0                # [Meters]
        default_left_fov = -180.0           # [°]
        default_right_fov = 180.0           # [°]
        default_upper_fov = 180.0           # [°]
        default_lower_fov = -180.0          # [°]
        default_min_corner_amount = 3       # [-]
        default_distance_variance = 10.0    # [Meters], 10 Meters based on the length of a truck

        # Check relevant attributes and set default values if not available or values are not set in parameter boundaries
        try: 
            self.range
            if self.range < 0:
                self.range = default_range
                self.node.logwarn(
                    "Range attribute for IdealObjectSensor is not in parameter boundaries! Using default value of {} meters.".format(self.range)
                )
        except:
            self.range = default_range
            self.node.logwarn(
                "No range attribute found for IdealObjectSensor. Using default value of {} meters.".format(self.range)
            )
        try:
            self.left_fov
            if self.left_fov < -180 or self.left_fov > 0:
                self.left_fov = default_left_fov
                self.node.logwarn(
                    "Left FOV attribute for IdealObjectSensor is not in parameter boundaries! Using default value of {} °.".format(self.left_fov)
                )
        except:
            self.left_fov = default_left_fov
            self.node.logwarn(
                "No left FOV attribute found for IdealObjectSensor. Using default value of {} °.".format(self.left_fov)
            )
        try:
            self.right_fov
            if self.right_fov < 0 or self.right_fov > 180:
                self.right_fov = default_right_fov
                self.node.logwarn(
                    "Right FOV attribute for IdealObjectSensor is not in parameter boundaries! Using default value of {} °.".format(self.right_fov)
                )
        except:
            self.right_fov = default_right_fov
            self.node.logwarn(
                "No right FOV attribute found for IdealObjectSensor. Using default value of {} °.".format(self.right_fov)
            )
        try:
            self.upper_fov
            if self.upper_fov < 0 or self.upper_fov > 180:
                self.upper_fov = default_upper_fov
                self.node.logwarn(
                    "Upper FOV attribute for IdealObjectSensor is not in parameter boundaries! Using default value of {} °.".format(self.upper_fov)
                )
        except:
            self.upper_fov = default_upper_fov
            self.node.logwarn(
                "No upper FOV attribute found for IdealObjectSensor. Using default value of {} °.".format(self.upper_fov)
            )
        try:
            self.lower_fov
            if self.lower_fov < -180 or self.lower_fov > 0:
                self.lower_fov = default_lower_fov
                self.node.logwarn(
                    "Lower FOV attribute for IdealObjectSensor is not in parameter boundaries! Using default value of {} °.".format(self.lower_fov)
                )
        except:
            self.lower_fov = default_lower_fov
            self.node.logwarn(
                "No lower FOV attribute found for IdealObjectSensor. Using default value of {} °.".format(self.lower_fov)
            )
        try:
            self.min_corner_amount
            if self.min_corner_amount < 1 or self.min_corner_amount > 8:
                self.min_corner_amount = default_min_corner_amount
                self.node.logwarn(
                    "Minimal corner amount attribute for IdealObjectSensor is not in parameter boundaries! Using default value of {} corners.".format(self.min_corner_amount)
                )
        except:
            self.min_corner_amount = default_min_corner_amount
            self.node.logwarn(
                "No minimal corner amount attribute found for IdealObjectSensor. Using default value of {} corners.".format(self.min_corner_amount)
            )
        try:
            self.distance_variance
            if self.distance_variance < 0:
                self.distance_variance = default_distance_variance
                self.node.logwarn(
                    "Distance variance attribute for IdealObjectSensor is not in parameter boundaries! Using defaut value of {} meters."-format(self.distance_variance)
                )
        except:
            self.distance_variance = default_distance_variance
            self.node.logwarn(
                "No distance variance attribute for distance measurement found for IdealObjectSensor. Using default value of {} Meters.".format(self.distance_variance)
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
    
    def calculate_azimuth_elevation(self, target_point_in_sensor_frame, distance):
        # Get location vaules of pose
        dx = target_point_in_sensor_frame.x
        dy = target_point_in_sensor_frame.y
        dz = target_point_in_sensor_frame.z

        # Calculate azimuth between target and sensor based on sensor KOS
        azimut_rad = math.atan2(dy, dx)
        azimut_deg = math.degrees(azimut_rad)

        # Calculate elevation between target and sensor based on sensor KOS
        try:
            elevation_rad = math.asin(dz/distance)
        except ValueError as error:
            raise error
        elevation_deg = math.degrees(elevation_rad)
            
        return azimut_deg, elevation_deg
    
    def check_visibility(self, carla_location_target_in_carla_map, ros_corners_in_sensor_frame, carla_corners_in_carla_map, carla_location_sensor_in_carla_map, timestamp):

        # FILTER 1
        # Calculate distance between sensor and target
        distance = carla_location_sensor_in_carla_map.distance(carla_location_target_in_carla_map)

        # Filter objects that are far outside the sensor range (including distance variance)
        if abs(distance-self.distance_variance) > self.range:
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
            # Get correct Position in ROS environemt
            ros_corner_pointstamped = ros_corners_in_sensor_frame[corner[0]]
            # Calculate azimuth and elevation
            azimuth_deg, elevation_deg = self.calculate_azimuth_elevation(ros_corner_pointstamped.point, corner[2])
            
            # Check if corner is inside the sensor FOV
            if azimuth_deg < self.left_fov: continue
            if azimuth_deg > self.right_fov: continue
            if elevation_deg > self.upper_fov: continue
            if elevation_deg < self.lower_fov: continue

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
                    # Skip hit points with the labe "NONE"
                    if hit_point.label is carla.CityObjectLabel.NONE:
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

        # Get ROS transform from idealIbjectSensor to carla_map and vice versa
        sensor_frame = self.get_prefix()
        time_latest_tf = Time(seconds=0)
        duration_timeout = Duration(seconds=0)
        try:
            ros_tf_sensor_to_carla_map = self.tf_buffer.lookup_transform(sensor_frame, 'carla_map' , time_latest_tf, duration_timeout)
            ros_tf_carla_map_to_sensor = self.tf_buffer.lookup_transform('carla_map', sensor_frame, time_latest_tf, duration_timeout)
        except:
            self.node.loginfo("{}: Could not transform {} to {} at the Frame {}".format(
                self.__class__.__name__, sensor_frame, 'carla_map', frame))
            return

        # Convert ROS Translation from carla_map to sensor into geometry_msgs/Point
        ros_point_sensor_in_carla_map = Point(
            x=ros_tf_carla_map_to_sensor.transform.translation.x,
            y=ros_tf_carla_map_to_sensor.transform.translation.y,
            z=ros_tf_carla_map_to_sensor.transform.translation.z
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
                    ros_corners_in_sensor_frame = self.convert_target_corner(carla_corners_in_carla_map, ros_tf_sensor_to_carla_map)

                    # Check visibility of the target
                    if self.check_visibility(carla_location_target_in_carla_map, ros_corners_in_sensor_frame, carla_corners_in_carla_map, carla_location_sensor_in_carla_map, timestamp):
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
                        ros_corners_in_sensor_frame = self.convert_target_corner(carla_corners_in_carla_map, ros_tf_sensor_to_carla_map)

                        # Check visibility of the target
                        if self.check_visibility(carla_location_target_in_carla_map, ros_corners_in_sensor_frame, carla_corners_in_carla_map, carla_location_sensor_in_carla_map, timestamp):
                            vehicle_obj = self._get_vehicle_from_environment_objects(vehicle, object_value)
                            ros_objects.objects.append(vehicle_obj)

        self.object_publisher.publish(ros_objects)