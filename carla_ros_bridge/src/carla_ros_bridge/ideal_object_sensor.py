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

from tf2_geometry_msgs import do_transform_pose, do_transform_point

import math

from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import Point, PointStamped

import ctypes # CHECK

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

        # Check relevant attributes and set default values if not available
        try: 
            self.range
        except:
            self.range = 15.0
            self.node.logwarn(
                "No range attribute found for IdealObjectSensor. Using default value of {} meters.".format(self.range)
            )
        try:
            self.left_fov
        except:
            self.left_fov = -180.0
            self.node.logwarn(
                "No left FOV attribute found for IdealObjectSensor. Using default value of {} °.".format(self.left_fov)
            )
        try:
            self.right_fov
        except:
            self.right_fov = 180.0
            self.node.logwarn(
                "No right FOV attribute found for IdealObjectSensor. Using default value of {} °.".format(self.right_fov)
            )
        try:
            self.upper_fov
        except:
            self.upper_fov = 180.0
            self.node.logwarn(
                "No upper FOV attribute found for IdealObjectSensor. Using default value of {} °.".format(self.upper_fov)
            )
        try:
            self.lower_fov
        except:
            self.lower_fov = -180.0
            self.node.logwarn(
                "No lower FOV attribute found for IdealObjectSensor. Using default value of {} °.".format(self.lower_fov)
            )
        try:
            self.min_corner_amount
        except:
            self.min_corner_amount = 3
            self.node.logwarn(
                "No minimal corner amount attribute found for IdealObjectSensor. Using default value of {} corners.".format(self.min_corner_amount)
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
    
    def calculate_distance(self, coordinates):
        # Calculate distance between coordinates and sensor
        return math.sqrt(coordinates.x**2 + coordinates.y**2 + coordinates.z**2)

    def check_visibility(self, target_pose_in_sensor_frame, corners_in_sensor_frame, carla_corners_in_carla_map, carla_location_sensor_in_carla_map, timestamp, id, is_actor):

        # Calculate distance between sensor and target
        distance = self.calculate_distance(target_pose_in_sensor_frame.position)

        # Set distance variance in [Meters]
        dinstance_variance = 10.0

        # Filter objects that are far outside the sensor range
        if abs(distance-dinstance_variance) > self.range:
            if is_actor: # CHECK
                print(f"ID {id}: Vehicle outside of sensor range!") # CHECK
            return False

        # Filter corners that are outside the sensor range and return if not enough corners are visible
        corner_list_distance = list()

        for corner_num, corner in enumerate(corners_in_sensor_frame):
            corner_distance = self.calculate_distance(corner.point)
            if corner_distance > self.range:
                continue
            else:
                corner_list_distance.append([corner_num, corner.point, corner_distance])

        if len(corner_list_distance) < self.min_corner_amount:
            if is_actor: # CHECK
                print(f"ID {id}: Too many corners outside of sensor range!") # CHECK
            return False

        # Filter corners that are covered by other objects and return if not enough corners are visible
        corner_list_not_covered = list()

        for corner in corner_list_distance:
            carla_corner_location = carla_corners_in_carla_map[corner[0]]
            hit_points = self.world.cast_ray(carla_location_sensor_in_carla_map, carla_corner_location)
            if hit_points:
                hit_point = hit_points[0]
                distance_hit_to_corner = hit_point.location.distance(carla_corner_location)
                if distance_hit_to_corner > 0.5:
                    # if is_actor: # CHECK
                    #     print(f"Sensor Location: {carla_location_sensor_in_carla_map}") # CHECK
                    #     print(f"ID {id} corner location: {carla_corner_location}") # CHECK
                    #     print(f"ID {id} hit points: {hit_points}") # CHECK
                    continue
            corner_list_not_covered.append(corner)
        
        if len(corner_list_not_covered) < self.min_corner_amount:
            if is_actor: # CHECK
                print(f"ID {id}: Too many corners are covered!") # CHECK
            return False

        # Filter corners that are outside the fov and return if not enough corners are visible
        corner_list_fov = list()

        for corner in corner_list_not_covered:
            # Calculate azimuth and elevation
            azimuth_deg, elevation_deg = self.calculate_azimuth_elevation(corner[1], corner[2])
            # Check, if point is inside the fov of the sensor
            if azimuth_deg < self.left_fov:
                continue
            elif azimuth_deg > self.right_fov:
                continue
            elif elevation_deg > self.upper_fov:
                continue
            elif elevation_deg < self.lower_fov:
                continue
            corner_list_fov.append(corner)
                
        if len(corner_list_fov) < self.min_corner_amount:
            if is_actor: # CHECK
                print(f"ID {id}: Too many corners are outside sensor fov!") # CHECK
            return False

        # Object is in distance, in fov and directly visible
        # if is_actor: # CHECK
        #     print(f"ID {id}: Vehicle is in sensor view!") # CHECK
        return True    

    def calculate_azimuth_elevation(self, target_point_in_sensor_frame, distance):
        # Get location vaules of pose
        dx = target_point_in_sensor_frame.x
        dy = target_point_in_sensor_frame.y
        dz = target_point_in_sensor_frame.z

        # Calculate azimuth between target and sensor based on sensor KOS
        azimut_rad = math.atan2(dy, dx)
        azimut_deg = math.degrees(azimut_rad)

        # Calculate elevation between target and sensor based on sensor KOS
        elevation_rad = math.asin(dz/distance)
        elevation_deg = math.degrees(elevation_rad)

        return azimut_deg, elevation_deg

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
    
    def point_to_pointstamped(self, point):
        # Convert geometry_msgs/Point to geometry_msgs/PointStamped
        point_stamped = PointStamped()
        point_stamped.point = point

        return point_stamped
    
    def publish_tf(self, timestamp):
        # Publish transform of idealObjectSensor
        transform =self.get_ros_transform(timestamp)
        try:
            self._tf_broadcaster.sendTransform(transform)
        except roscomp.exceptions.ROSException:
            if roscomp.ok():
                self.node.logwarn("Sensor {} failed to send transform.".fromat(self.uid))

    def convert_target_corner(self, carla_corners_in_carla_map, ros_tf_carla_map_to_sensor):

        # Convert target corners from CARLA.Location to geometry_msgs/PointStamped (ROS)
        ros_corners_in_carla_map_point = [trans.carla_location_to_ros_point(corner) for corner in carla_corners_in_carla_map]
        ros_corners_in_carla_map_pointstamped = [self.point_to_pointstamped(corner) for corner in ros_corners_in_carla_map_point]

        # Transform target corners from carla_map frame to sensor frame
        corners_in_sensor_frame = [do_transform_point(corner, ros_tf_carla_map_to_sensor) for corner in ros_corners_in_carla_map_pointstamped]
        
        return corners_in_sensor_frame

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

        # Get ROS transform from idealIbjectSensor in carla_map
        sensor_frame = self.get_prefix()
        time_latest_tf = Time(seconds=0)
        duration_timeout = Duration(seconds=0)
        try:
            ros_tf_carla_map_to_sensor = self.tf_buffer.lookup_transform(sensor_frame, 'carla_map', time_latest_tf, duration_timeout)
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
        # Convert ROS Point to CARLA Location
        carla_location_sensor_in_carla_map = trans.ros_point_to_carla_location(ros_point_sensor_in_carla_map)

        # Iterate over all dynamic actors
        for actor_id in self.actor_list.keys():

            # Currently only vehicles and walkers are added to the object array
            if self.parent is None or self.parent.uid != actor_id:
                actor = self.actor_list[actor_id]
                if isinstance(actor, Vehicle) or isinstance(actor, Walker):

                    # Get ROS target pose in carla_map and transform to ROS sensor frame
                    ros_target_pose_in_carla_map = actor.get_current_ros_pose()
                    ros_target_pose_in_sensor_frame = do_transform_pose(ros_target_pose_in_carla_map, ros_tf_carla_map_to_sensor)
                    
                    # Get corners from target BoundingBox and convert location from CARLA carla_map into ROS sensor frame
                    carla_tf_carla_map_to_target = actor.carla_actor.get_transform()
                    
                    bounding_box = actor.carla_actor.bounding_box
                    carla_corners_in_carla_map = bounding_box.get_world_vertices(carla_tf_carla_map_to_target)

                    corners_in_sensor_frame = self.convert_target_corner(carla_corners_in_carla_map, ros_tf_carla_map_to_sensor)

                    # Check visibility of the target
                    if self.check_visibility(ros_target_pose_in_sensor_frame, corners_in_sensor_frame, carla_corners_in_carla_map, carla_location_sensor_in_carla_map, timestamp, actor_id, True):
                        ros_objects.objects.append(actor.get_object_info())

        # Iterate over all static vehicles
        if(self.node.parameters['publish_static_vehicles']):
            for object_key, object_value in self.OBJECT_LABELS.items():

                static_vehicles = self.world.get_environment_objects(object_key)

                for vehicle in static_vehicles:
                    # Take only vehicles with bounding_box attribute set
                    if hasattr(vehicle, "bounding_box"):

                        # Get target pose in carla_map and transform to sensor frame
                        ros_target_pose_in_carla_map = trans.carla_transform_to_ros_pose(vehicle.transform)
                        ros_target_pose_in_sensor_frame = do_transform_pose(ros_target_pose_in_carla_map, ros_tf_carla_map_to_sensor)

                        # Get corners from target BoundingBox and convert location from CARLA carla_map to ROS sensor frame
                        bounding_box = vehicle.bounding_box
                        carla_corners_in_carla_map = bounding_box.get_local_vertices()

                        corners_in_sensor_frame = self.convert_target_corner(carla_corners_in_carla_map, ros_tf_carla_map_to_sensor)

                        # Check visibility of the target
                        if self.check_visibility(ros_target_pose_in_sensor_frame, corners_in_sensor_frame, carla_corners_in_carla_map, carla_location_sensor_in_carla_map, timestamp, ctypes.c_uint32(vehicle.id).value, False):
                            vehicle_obj = self._get_vehicle_from_environment_objects(vehicle, object_value)
                            ros_objects.objects.append(vehicle_obj)

        self.object_publisher.publish(ros_objects)