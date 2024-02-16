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

from carla_ros_bridge.vehicle import Vehicle
from carla_ros_bridge.walker import Walker

from derived_object_msgs.msg import ObjectArray, Object
from shape_msgs.msg import SolidPrimitive
from transforms3d.euler import euler2quat
import math

from carla_ros_bridge.object_sensor import ObjectSensor

small_distance_threshold = 7

class IntelligentObjectSensor(ObjectSensor):

    """
    Intelligent object sensor
    """

    def __init__(self, uid, name, parent, node, actor_list, world, attributes):
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
        
        super(IntelligentObjectSensor, self).__init__(uid=uid,
                                                      name=name,
                                                      parent=parent,
                                                      node=node,
                                                      actor_list=actor_list, 
                                                      world=world)
        self.node = node
        self.attributes = attributes
        self.object_publisher = node.new_publisher(ObjectArray,
                                                   self.get_topic_prefix(),
                                                   qos_profile=10)
    
    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        super(IntelligentObjectSensor, self).destroy()
        self.actor_list = None
        self.node.destroy_publisher(self.object_publisher)


    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.visible_objects"

    def check_visibility(self, ego_vehicle, target, actor_id): 
        # Get the yaw angle of the source vehicle 
        source_yaw = ego_vehicle.carla_actor.get_transform().rotation.yaw

        # Get the location of the source parent vehicle of Intelligent Object Sensor and the target
        ego_vehicle_location = ego_vehicle.carla_actor.get_location()
        target_location = target.carla_actor.get_location()
        
        target_transform = target.carla_actor.get_transform()
        target_bounding_box = target.carla_actor.bounding_box
        # Get the locations of the corners of the bounding box in the world coordinates 
        corners = target_bounding_box.get_world_vertices(target_transform)
        # Calculate the distance from the vehicle's center to each corner
        distances = [self.distance_between_points(target_location, corner) for corner in corners]
        
        # Calculate the Euclidean distance between the ego and the target 
        distance = ego_vehicle_location.distance(target_location)

        # Check if the target is inside the range of the sensor 
        if distance <= self.attributes["range"]:
            """
            # Calculate the azimuth and elevation angle between the Ego and target Vehicles
            target_azimuth, target_elevation = self.calculate_azimuth_and_elevation(ego_vehicle_location, target_location, source_yaw)
            
            print(f"{actor_id} HAS AZIMUTH {target_azimuth} AND ELEVATION {target_elevation}")
            if self.attributes["min_azimuth"] <= target_azimuth <= self.attributes["max_azimuth"] and self.attributes["min_elevation"] <= target_elevation <= self.attributes["max_elevation"]:  
            """
            visible_corner_count = 0 
            for i, corner in enumerate(corners): 
                # Perform Raycasting towards each corner 
                hit_points = self.world.cast_ray(ego_vehicle_location, corner)

                # If a ray hits something, it should return at least one hit_point, take the first for simplicity 
                if hit_points: 
                    hit_point = hit_points[0]

                    # Calculate the distance between the hit_point's location and its corner's location 
                    hit_distance = hit_point.location.distance(corner)
                    
                    center_to_corner_distance = distances[i]

                    # print(f"distance of the vehicle {actor_id} is {distance} and its hit distance is {hit_distance}")
                    # Use a small threshold to check if the ray actually hit the target 
                    if hit_distance <= center_to_corner_distance + small_distance_threshold: 
                        visible_corner_count += 1

            # Consider an object is visible if at least half of its corners are visible
            if visible_corner_count: 
                return distance, True
        
        return distance, False 
        
    def distance_between_points(self, point1, point2): 
        """Calculate the distance between two carla.Location points"""
        dx = point2.x - point1.x
        dy = point2.y - point1.y
        dz = point2.z - point1.z 
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def calculate_azimuth_and_elevation(self, source_location, target_location, source_yaw):  
        # Calculate the vector from source to target 
        delta_x = target_location.x - source_location.x
        delta_y = target_location.y - source_location.y
        delta_z = target_location.z - source_location.z 

        # Calculate the horizontal distance from source to target
        horizontal_distance = math.sqrt(delta_x**2 + delta_y**2)

        # Calculate the angle in radians using atan2 for azimuth 
        # azimuth_rad = math.atan2(-delta_y, delta_x)
        azimuth_rad = math.atan2(delta_x, delta_y)

        # Calculate the angle in radians using atan2 for elevation 
        elevation_rad = math.atan2(delta_z, horizontal_distance)

        # Convert the angle from radians to degrees 
        azimuth_deg = math.degrees(azimuth_rad)
        if azimuth_deg < 0: 
            azimuth_deg += 360
        elevation_deg = math.degrees(elevation_rad)

        return azimuth_deg, elevation_deg 

    def frustum_culling(self, ego_vehicle, target): 
        # Get the locations of ego vehicle and the target 
        ego_vehicle_location = ego_vehicle.carla_actor.get_location()
        target_location = target.carla_actor.get_location()

        # Calculate the distance 
        distance = ego_vehicle_location.distance(target_location)

        # Check if the target is within the sensor's threshold range
        if distance > self.attributes["range"]: 
            return distance, False 
            
        # Define frustum planes 
        half_fov_horiz = (self.attributes["max_azimuth"] - self.attributes["min_azimuth"]) / 2.0
        half_fov_vert = (self.attributes["max_elevation"] - self.attributes["min_elevation"]) / 2.0

        right_normal = (
            math.sin(math.radians(half_fov_horiz)), 
            math.cos(math.radians(half_fov_horiz)), 
            0 
        )
        left_normal = (
            -math.sin(math.radians(half_fov_horiz)), 
            math.cos(math.radians(half_fov_horiz)), 
            0
        )
        top_normal = (
        0,
        math.sin(math.radians(half_fov_vert)),
        -math.cos(math.radians(half_fov_vert))
        )
        bottom_normal = (
            0,
            math.sin(math.radians(-half_fov_vert)),
            math.cos(math.radians(-half_fov_vert))
        )

        # Near and far planes might be defined based on sensor specifics. 
        # For simplicity, assuming normals straight forward.
        near_normal = (0, 1, 0)
        far_normal = (0, -1, 0)
        
        frustum_planes = [left_normal, right_normal, top_normal, bottom_normal, near_normal, far_normal]
        print(frustum_planes)
        # Step 2: Check for bounding box intersection with frustum
        bounding_box = target.carla_actor.bounding_box
        for plane_normal in frustum_planes:
            # This check uses the separating axis theorem to determine if the bounding box intersects the frustum.
            if not self._bounding_box_intersects_plane(bounding_box, plane_normal, ego_vehicle_location):
                return distance, False

        return distance, True
    
    def _bounding_box_intersects_plane(self, bounding_box, plane_normal, point_on_plane):
        # Separating Axis Theorem (SAT)
        # This function checks if the given bounding box intersects a plane defined by its normal and a point on the plane.
        
        # The center of the bounding box
        center = bounding_box.location
        half_extents = (bounding_box.extent.x, bounding_box.extent.y, bounding_box.extent.z)

        # Positive vertex determined by the plane normal
        p_vertex = [
            center.x + (-half_extents[i] if plane_normal[i] < 0 else half_extents[i])
            for i in range(3)
        ]

        # Distance from the positive vertex to the plane
        distance = (
            plane_normal[0] * (p_vertex[0] - point_on_plane.x) + 
            plane_normal[1] * (p_vertex[1] - point_on_plane.y) +
            plane_normal[2] * (p_vertex[2] - point_on_plane.z)
        )

        return distance >= 0 

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        On update carla_map sends:
        - tf global frame
        :return:
        """
        ros_objects = ObjectArray()
        ros_objects.header = self.get_msg_header(frame_id="carla_map", timestamp=timestamp)

        if not self.parent: 
            return 
        
        """       
            - Get the vehicle that the intelligence sensor is appended
            - This can be either Ego Vehicle or Hero Vehicle based on the sensors.json definitions
        """
        ego_vehicle = self.actor_list[self.parent.uid]  
        """
        # Print the locations of the Ego Vehicle and Hero Vehicle
        if ego_vehicle.uid == 47: 
            print(f"The location of the ego vehicle is {ego_vehicle.carla_actor.get_location()}")
        if ego_vehicle.uid == 50: 
            print(f"The location of the hero vehicle is {ego_vehicle.carla_actor.get_location()}")
        """
        for actor_id in self.actor_list.keys():
            # currently only Vehicles and Walkers are added to the object array
            if self.parent is None or self.parent.uid != actor_id:
                actor = self.actor_list[actor_id]
                if isinstance(actor, Vehicle) or isinstance(actor, Walker):
                    # distance, in_frustum = self.frustum_culling(ego_vehicle, actor)
                    distance, visible = self.check_visibility(ego_vehicle, actor, actor_id)
                    if visible : 
                        print(f"ego vehicle is {ego_vehicle.uid}, and the distance of the target with an id of {actor_id} is {distance}")
                        ros_objects.objects.append(actor.get_object_info())

        self.object_publisher.publish(ros_objects)