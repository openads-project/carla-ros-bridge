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

from geometry_msgs.msg import Pose, Quaternion, TransformStamped

from carla_ros_bridge.object_sensor import ObjectSensor

from carla.libcarla import Location, Rotation, Vector3D

try:
    import queue
except ImportError:
    import Queue as queue

import carla_common.transforms as trans
import ros_compatibility as roscomp

import tf2_ros

from tf2_geometry_msgs import do_transform_pose

try:
    from tf_transformations import euler_from_quaternion, quaternion_from_euler
    # import tf_transformations
except ImportError:
    from tf.transformations import euler_from_quaternion, quaternion_from_euler
# from inspect import getmembers, isfunction

import math
import numpy as np

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
        # print(getmembers(tf_transformations, isfunction))
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
        self.tf_buffer = tf2_ros.Buffer()
        if ROS_VERSION == 1:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster()
        elif ROS_VERSION == 2:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(node)
        
        # Extract (relative) spawn pose
        self.relative_spawn_pose = relative_spawn_pose
        # Extract spawn pose and convert (relative) position to carla.Location
        self.relative_position = Location(
            x=relative_spawn_pose.position.x,
            y=relative_spawn_pose.position.y,
            z=relative_spawn_pose.position.z
            )
        # Extract spawn orientation and convert Rotation to carla.Rotation
        orientation_list = [
            relative_spawn_pose.orientation.x,
            relative_spawn_pose.orientation.y,
            relative_spawn_pose.orientation.z,
            relative_spawn_pose.orientation.w
            ]
        (roll_rad, pitch_rad, yaw_rad) = euler_from_quaternion(orientation_list)
        roll_deg = math.degrees(roll_rad)
        pitch_deg = math.degrees(pitch_rad)
        yaw_deg = math.degrees(yaw_rad)
        self.relative_rotation = Rotation(pitch=pitch_deg, yaw=yaw_deg, roll=roll_deg)
        # Extract orientation
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
    
    def check_visibility(self, sensor_location, target_location, target_pose_in_sensor_frame, timestamp, tp): 

        # Calculate the euclidean distance between sensor and target
        # print(f"sensor_location: {sensor_location}, target_location: {target_location}")
        # distance = sensor_location.distance(target_location)

        # Calculate azimuth and elevation between sensor and target
        azimut_deg, elevation_deg, distance = self.calculate_azimut_elevation_distance(target_pose_in_sensor_frame, timestamp, tp)

        # Check if the target is inside the range and fov of the sensor
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
            # if distance < 10:
            #     print(f"Distance: {round(distance, 2)}, ID: {id}, dx: {round(dx,2)}, dy: {round(dy, 2)}, dz: {round(dz, 2)}, Azimuth: {round(azimuth_deg, 2)}, Elevation: {round(elevation_deg, 2)}")
            return True
    

    def calculate_azimut_elevation_distance(self, target_pose_in_sensor_frame, timestamp, tp):

        # Get location vaules of pose
        dx = target_pose_in_sensor_frame.position.x
        dy = target_pose_in_sensor_frame.position.y
        dz = target_pose_in_sensor_frame.position.z

        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        # print(f"dx: {dx}, dy: {dy}, dz: {dz}, distance: {distance}, type: {tp}")

        # Calculate azimuth
        azimut_rad = math.atan(dy/dx)
        azimut_deg = math.degrees(azimut_rad)

        # Calculate elevation
        elevation_rad = math.asin(dz/distance)
        elevation_deg = math.degrees(elevation_rad)

        return azimut_deg, elevation_deg, distance
    

    def get_ros_transform(self, timestamp):
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

        sensor_frame = self.get_prefix()
        # print(f"timestamp: {timestamp}")
        # print(f"Type timestamp: {type(timestamp)}")
        # timestamp_time = datetime(timestamp)
        time_latest_tf = Time(seconds=0)
        duration_timeout = Duration(seconds=0)
        # tf_carla_map_to_sensor = self.tf_buffer.lookup_transform(sensor_frame, 'carla_map', time_latest_tf, duration_timeout)
        try:
            tf_carla_map_to_sensor = self.tf_buffer.lookup_transform(sensor_frame, 'carla_map', time_latest_tf, duration_timeout)
            print("YES!")
        except:
            print("lookupTransform not working!")
        # print(f"tf_carla_map_to_sensor: {tf_carla_map_to_sensor}")
        # Construct sensor location

        if not self.parent:
            # Get location for idealObjectSensor without vehicle parent
            sensor_location = self.relative_position

            # Get pose of idealObjectSensor without vehicle parent
            sensor_pose = self.relative_spawn_pose

            # Transform sensor Pose to TransformStamped
            tf_carla_map_to_sensor = TransformStamped()
            tf_carla_map_to_sensor.transform.translation.x = sensor_pose.position.x
            tf_carla_map_to_sensor.transform.translation.y = sensor_pose.position.y
            tf_carla_map_to_sensor.transform.translation.z = sensor_pose.position.z
            tf_carla_map_to_sensor.transform.rotation.x = sensor_pose.orientation.x
            tf_carla_map_to_sensor.transform.rotation.y = sensor_pose.orientation.y
            tf_carla_map_to_sensor.transform.rotation.z = sensor_pose.orientation.z
            tf_carla_map_to_sensor.transform.rotation.w = sensor_pose.orientaiton.w

        else:
            """       
                - Get the vehicle that the IdealObjectSensor is appended
                - This can be either ego-vehicle or hero-vehicle based on the sensors.json definitions
                - Calculate position of the IdealObjectSensor located in vehicle
            """
            # Get ego_vehicle location and rotation (in degrees)
            ego_vehicle = self.actor_list[self.parent.uid]
            ego_vehicle_transform = ego_vehicle.carla_actor.get_transform()

            # Calculate location and orientation for idealObjectSensor including vehicle parent
            sensor_location = self.relative_position + ego_vehicle_transform.location
            sensor_orientation_degree = [
                self.relative_rotation.pitch + ego_vehicle_transform.rotation.pitch,
                self.relative_rotation.yaw + ego_vehicle_transform.rotation.yaw,
                self.relative_rotation.roll + ego_vehicle_transform.rotation.roll
            ]

            # Convert sensor orientation from degrees to rad
            sensor_orientation_rad = np.radians(sensor_orientation_degree)

            # Convert sensor orientation from rad to quaternion
            sensor_orientation = quaternion_from_euler(
                sensor_orientation_rad[0],
                sensor_orientation_rad[1],
                sensor_orientation_rad[2]
            )

            tf_carla_map_to_sensor = TransformStamped()
            tf_carla_map_to_sensor.transform.translation.x = sensor_location.x
            tf_carla_map_to_sensor.transform.translation.y = sensor_location.y
            tf_carla_map_to_sensor.transform.translation.z = sensor_location.z
            tf_carla_map_to_sensor.transform.rotation.x = sensor_orientation[0]
            tf_carla_map_to_sensor.transform.rotation.y = sensor_orientation[1]
            tf_carla_map_to_sensor.transform.rotation.z = sensor_orientation[2]
            tf_carla_map_to_sensor.transform.rotation.w = sensor_orientation[3]
        
        # Iterate over all dynamic actors
        for actor_id in self.actor_list.keys():
            
            # Currently only vehicles and walkers are added to the object array
            if self.parent is None or self.parent.uid != actor_id:
                actor = self.actor_list[actor_id]
                if isinstance(actor, Vehicle) or isinstance(actor, Walker):
                    
                    # Get target location
                    target_location = actor.carla_actor.get_location()

                    # Get target pose in carla map
                    target_pose_in_carla_map = actor.get_current_ros_pose()

                    # Transform target pose from carla map to sensor frame
                    target_pose_in_sensor_frame = do_transform_pose(target_pose_in_carla_map, tf_carla_map_to_sensor)

                    # Check visibility of the target
                    if self.check_visibility(sensor_location, target_location, target_pose_in_sensor_frame, timestamp, 'actor'):
                        ros_objects.objects.append(actor.get_object_info())

        # Iterate over all static vehicles
        if(self.node.parameters['publish_static_vehicles']):
            for object_key, object_value in self.OBJECT_LABELS.items():
                static_vehicles = self.world.get_environment_objects(object_key)

                for vehicle in static_vehicles:
                    # Take only vehicles with bounding_box attribute set
                    if hasattr(vehicle, "bounding_box"):
                        
                        # Get target location in carla map
                        target_location = vehicle.transform.location

                        # Get target rotation in carla map
                        target_rotation_degree = [
                            vehicle.transform.rotation.pitch,
                            vehicle.transform.rotation.yaw,
                            vehicle.transform.rotation.roll
                        ]
                        target_rotation_rad = np.radians(target_rotation_degree)

                        # Convert rotation in rad to orientation in quaternion
                        target_orientation = quaternion_from_euler(
                            target_rotation_rad[0],
                            target_rotation_rad[1],
                            target_rotation_rad[2]
                        )

                        # Create target pose in carla map
                        target_pose_in_carla_map = Pose()
                        target_pose_in_carla_map.position.x = target_location.x
                        target_pose_in_carla_map.position.y = target_location.y
                        target_pose_in_carla_map.position.z = target_location.z
                        target_pose_in_carla_map.orientation.x = target_orientation[0]
                        target_pose_in_carla_map.orientation.y = target_orientation[1]
                        target_pose_in_carla_map.orientation.z = target_orientation[2]
                        target_pose_in_carla_map.orientation.w = target_orientation[3]

                        # Transform target pos from carla map to sensor frame
                        target_pose_in_sensor_frame = do_transform_pose(target_pose_in_carla_map, tf_carla_map_to_sensor)

                        # Check visibility of the target
                        if self.check_visibility(sensor_location, target_location, target_pose_in_sensor_frame, timestamp, 'static'):
                            vehicle_obj = self._get_vehicle_from_environment_objects(vehicle, object_value)
                            ros_objects.objects.append(vehicle_obj)

        self.object_publisher.publish(ros_objects)