#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
a sensor that reports the state of all traffic lights
"""

import math
import pyproj
import rclpy
from ros_compatibility.qos import QoSProfile, DurabilityPolicy
import numpy as np

from carla_ros_bridge.pseudo_actor import PseudoActor
from carla_ros_bridge.traffic import TrafficLight
import tf2_ros
from geometry_msgs.msg import Vector3

from carla_msgs.msg import (
    CarlaTrafficLightStatusList,
    CarlaTrafficLightInfoList
)

from carla_msgs.msg import CarlaTrafficLightStatus, CarlaTrafficLightInfo
from rclpy.node import Node
from carla.libcarla import LaneType
from carla.libcarla import LaneChange
from carla.libcarla import Location

from etsi_its_mapem_ts_msgs.msg import MAPEM
from etsi_its_mapem_ts_msgs.msg import IntersectionGeometry
from etsi_its_mapem_ts_msgs.msg import GenericLane
from etsi_its_mapem_ts_msgs.msg import Connection
from etsi_its_mapem_ts_msgs.msg import NodeListXY
from etsi_its_mapem_ts_msgs.msg import NodeXY
from etsi_its_mapem_ts_msgs.msg import LaneTypeAttributes

from etsi_its_spatem_ts_msgs.msg import IntersectionState
from etsi_its_spatem_ts_msgs.msg import MovementState
from etsi_its_spatem_ts_msgs.msg import MovementPhaseState
from etsi_its_spatem_ts_msgs.msg import MovementEvent

from visualization_msgs.msg import Marker, MarkerArray
from etsi_its_spatem_ts_msgs.msg import SPATEM



class TrafficLightsSensor(PseudoActor):
    """
    a sensor that reports the state of all traffic lights
    """

    def __init__(self, uid, name, parent, node, actor_list):
        """
        Constructor
        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying the sensor
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: CompatibleNode
        :param actor_list: current list of actors
        :type actor_list: map(carla-actor-id -> python-actor-object)
        """

        super(TrafficLightsSensor, self).__init__(uid=uid,
                                                  name=name,
                                                  parent=parent,
                                                  node=node)
        self.node = node
        self.actor_list = actor_list
        self.traffic_light_status = CarlaTrafficLightStatusList()
        self.traffic_light_actors = []

        self.traffic_lights_info_publisher = node.new_publisher(
            CarlaTrafficLightInfoList,
            self.get_topic_prefix() + "/info",
            qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.traffic_lights_status_publisher = node.new_publisher(
            CarlaTrafficLightStatusList,
            self.get_topic_prefix() + "/status",
            qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        
        self.etsi_mapem_publisher = node.new_publisher(
            MAPEM,
            "/carla/etsi_mapem",
            qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.etsi_spatem_publisher = node.new_publisher(
            SPATEM,
            "/carla/etsi_spatem",
            qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        
        self.marker_publisher = node.new_publisher(
            MarkerArray,
            "/carla/traffic_light_triggers",
            qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    
        self.initialize_junctions()

    def initialize_junctions(self):
        self.traffic_light_actors = []
        for actor_id in self.actor_list:
            actor = self.actor_list[actor_id]
            if isinstance(actor, TrafficLight):
                self.traffic_light_actors.append(actor)
                
        self.junctions = self.get_junctions(self.traffic_light_actors)       
        

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        super(TrafficLightsSensor, self).destroy()
        self.actor_list = None
        self.node.destroy_publisher(self.traffic_lights_info_publisher)
        self.node.destroy_publisher(self.traffic_lights_status_publisher)
        self.node.destroy_publisher(self.etsi_mapem_publisher)
        self.node.destroy_publisher(self.etsi_spatem_publisher)

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.traffic_lights"
    
    """
    Convert CARLA coordinates to latitude/longitude using PyProj and a reference point.
    
    Args:
        carla_x, carla_y: CARLA coordinates to convert
        projection_string: Projection string whichn is used to convert Carla coordinates to lat/lon coordinates
    
    Returns:
        latitude, longitude: WGS84 coordinates
    """
    def carla_to_latlon(projection_string, carla_x, carla_y):
        proj_xodr = pyproj.Proj(projparams=projection_string)
        lon, lat = proj_xodr(carla_x, carla_y, inverse=True)
        
        return lat, lon

    """
    Convert a CARLA LaneType into an Etsi LaneType
    
    Args:
        carla_x, carla_y: CARLA coordinates to convert
        projection_string: Projection string whichn is used to convert Carla coordinates to lat/lon coordinates
    
    Returns:
        latitude, longitude: WGS84 coordinates
    """
    @staticmethod
    def convert_lane_type(carla_lane_type : LaneType):
        lane_actions = {
            LaneType.NONE: 0,
            LaneType.Driving: LaneTypeAttributes.CHOICE_VEHICLE,
            LaneType.Stop: 0,
            LaneType.Shoulder: 0,
            LaneType.Biking: LaneTypeAttributes.CHOICE_BIKE_LANE,
            LaneType.Sidewalk: LaneTypeAttributes.CHOICE_SIDEWALK,
            LaneType.Border: 0,
            LaneType.Restricted: 0,
            LaneType.Parking: LaneTypeAttributes.CHOICE_PARKING,
            LaneType.Bidirectional: 0,
            LaneType.Median: LaneTypeAttributes.CHOICE_MEDIAN,
            LaneType.Special1: 0,
            LaneType.Special2: 0,
            LaneType.Special3: 0,
            LaneType.RoadWorks: 0,
            LaneType.Tram: 0,
            LaneType.Rail: 0,
            LaneType.Entry: 0,
            LaneType.Exit: 0,
            LaneType.OffRamp: 0,
            LaneType.OnRamp: 0,
            LaneType.Any: 0
        }
            
        return lane_actions[carla_lane_type]
    
    @staticmethod
    def convert_traffic_light_state(state : CarlaTrafficLightStatus):
        state_dictionary = {
            CarlaTrafficLightStatus.RED: MovementPhaseState.STOP_THEN_PROCEED,
            CarlaTrafficLightStatus.YELLOW: MovementPhaseState.PRE_MOVEMENT,
            CarlaTrafficLightStatus.GREEN: MovementPhaseState.PERMISSIVE_MOVEMENT_ALLOWED,
            CarlaTrafficLightStatus.OFF: MovementPhaseState.DARK,
            CarlaTrafficLightStatus.UNKNOWN: MovementPhaseState.DARK    
        }
        
        return state_dictionary[state]

    @staticmethod
    def add_node(lane, x, y):
        node = NodeXY()
        node.delta.node_xy1.x.value = (int)(x * 100)
        node.delta.node_xy1.y.value = (int)(y * 100)
        lane.node_list.nodes.array.append(node)
    
    def get_junctions(self, traffic_lights):
        junctions = {}
        map = self.node.carla_world.get_map()
        all_waypoints = map.generate_waypoints(0.5)
        
        for waypoint in all_waypoints:
            if waypoint.is_junction:
                # build the junction data structure with all it's traffic lights and in- and outgoing driving lanes
                junction_id = waypoint.junction_id
                
                if junction_id not in junctions:
                    print("Created junction with id: ", junction_id)
                    
                    # Try to get the actual junction object
                    junction_object = waypoint.get_junction()
                            
                    junctions[junction_id] = {
                        'junction_object': junction_object,
                        'traffic_lights': {},
                        'waypoints_tuple': []
                    }
                    
                    # get all waypoint tuples for a given junction
                    # a tuple represents a driving line from the entrance to a junction ingress -first tuple element) 
                    # to an exit (egress - second tuple element)
                    # an ingress lane can have an attached traffic light 
                    waypointTuples = junction_object.get_waypoints(LaneType.Driving)
                    
                    for waypointTuple in waypointTuples:
                        junctions[junction_id]['waypoints_tuple'].append(waypointTuple)
                        wp1, wp2 = waypointTuple
                        
                        (waypoint, traffic_light) = TrafficLightsSensor.get_affected_traffic_light_waypoint(traffic_lights, wp1.road_id)
                        
                        if (waypoint, traffic_light) != (None, None):
                            junctions[junction_id]['traffic_lights'][traffic_light.id] = traffic_light
                                            
        return junctions 
              
    
    def get_junctions_old(self, traffic_light_actors):
        junctions = {}
        count = 0
        print("____ Length of traffic light acotrs arr: ", len(traffic_light_actors))
        for actor in traffic_light_actors:
            if isinstance(actor, TrafficLight):
                traffic_light = actor
                # find a waypoint which is part of the junction
                waypoints_traffic_light = TrafficLightsSensor.get_waypoints_from_traffic_light(traffic_light) 
                
                if len(waypoints_traffic_light) == 0:
                    self.node.loginfo("Traffic light actor with id: {} has no affected lane waypoint".format(traffic_light.uid))
                    continue
                
                junction_waypoint = None
                
                for affected_waypoint in waypoints_traffic_light:
                    if affected_waypoint.is_junction:
                        junction_waypoint = affected_waypoint
                        break
                    
                
                nearest_junction = None
                
                if junction_waypoint == None:
                # If still no junction found, try to find the nearest junction
                    all_junctions = []
                    all_waypoints = self.node.carla_world.get_map().generate_waypoints(2.0)
                    min_distance = float('inf')
                    
                    for wp in all_waypoints:
                        if wp.is_junction and wp.get_junction() not in all_junctions:
                            all_junctions.append(wp.get_junction())
                            j = wp.get_junction()
                            tl_location = actor.get_info().transform.position
                            distance = (tl_location.x - j.bounding_box.location.x) ** 2 + (tl_location.y - j.bounding_box.location.y) ** 2
                            if distance < min_distance:
                                min_distance = distance
                                junction_waypoint = wp
                    
                if junction_waypoint != None:
                    count = count +1
                    # build the junction data structure with all it's traffic lights and in- and outgoing driving lanes
                    junction_id = junction_waypoint.junction_id
                    
                    # Create new junction entry if it doesn't exist
                    if junction_id not in junctions:
                        # Try to get the actual junction object
                        junction_object = junction_waypoint.get_junction()
                        
                        junctions[junction_id] = {
                            'junction_object': junction_object,
                            'traffic_lights': {},
                            'waypoints_tuple': []
                        }
                        
                        # get all waypoint tuples for a given junction
                        # a tuple represents a driving line from the entrance to a junction ingress -first tuple element) 
                        # to an exit (egress - second tuple element)
                        # an ingress lane can have an attached traffic light 
                        waypointTuples = junction_object.get_waypoints(LaneType.Driving)
                        
                        for waypointTuple in waypointTuples:
                            junctions[junction_id]['waypoints_tuple'].append(waypointTuple)
                        
                    junctions[junction_id]['traffic_lights'][traffic_light.uid] = traffic_light
                    
        print("final traffic light count: ", count)      
        return junctions
    
    @staticmethod
    def get_waypoints_from_traffic_light(traffic_light):
        return traffic_light.get_stop_waypoints()
        waypoints = traffic_light.get_affected_lane_waypoints()
        
        if (len(waypoints) == 0):
            waypoints = traffic_light.get_stop_waypoints()
        
        return waypoints
    
    
    @staticmethod
    def get_affected_traffic_light_waypoint(traffic_lights, road_id):
        
        for traffic_light in traffic_lights:
            waypoints = TrafficLightsSensor.get_waypoints_from_traffic_light(traffic_light.carla_actor)
            waypoint = waypoints[0]
            
            while waypoint is not None:
                if waypoint.is_junction:    
                    if road_id == waypoint.road_id:
                        return (waypoint, traffic_light.carla_actor)
                    
                    break
                
                # Get the next waypoint in the list
                waypoint = waypoint.next(0.5)[0]
            
            #for waypoint in waypoints:
            #    if waypoint.road_id == road_id:
            #        return (waypoint, traffic_light)
                
        return (None, None)
                    
    def publish_etsi_messages(self, traffic_light_actors):
        #junctions = self.get_junctions(traffic_light_actors)
        #self.debug_publish_traffic_information(junctions)
        mapem = TrafficLightsSensor.create_etsi_mapem_message(self.node.world_info.projection_string, self.junctions, traffic_light_actors, 10, 1.0)
        self.etsi_mapem_publisher.publish(mapem)
        
        spatem = TrafficLightsSensor.create_etsi_spatem_message(self.junctions)
        self.etsi_spatem_publisher.publish(spatem)
        
    @staticmethod 
    def create_junction_lane(is_ingress, waypoint, junctionPosX, junctionPosY, lane_segments_count, lane_segments_distance):
        # create ingress line for
        generic_lane_ingress = GenericLane()
        generic_lane_ingress.lane_id.value = waypoint.road_id   
        generic_lane_ingress._lane_attributes.lane_type.choice = TrafficLightsSensor.convert_lane_type(waypoint.lane_type)

        # build the bitstring for ingress line: 128 encodes ingress and 192 encodes egress in big endian format
        generic_lane_ingress.lane_attributes.directional_use.value.append(128 if is_ingress else 64)
        generic_lane_ingress.lane_attributes.directional_use.bits_unused = 6
        
        # lane consists of a nodelist of 2 nodes
        generic_lane_ingress.node_list = NodeListXY()
        generic_lane_ingress.node_list.choice = NodeListXY.CHOICE_NODES

        pos_abs_x = waypoint.transform.location.x
        pos_abs_y = -waypoint.transform.location.y # convert y carla into ros frame
        
        pos_rel_junction_x = pos_abs_x - junctionPosX
        pos_rel_junction_y = pos_abs_y - junctionPosY

        TrafficLightsSensor.add_node(generic_lane_ingress, pos_rel_junction_x, pos_rel_junction_y)
            
        last_wp = waypoint
        last_pos_x = pos_abs_x
        last_pos_y = pos_abs_y
        
        # create an egress/ingress line with a given length
        for i in range(lane_segments_count):
            next_wps = last_wp.previous(lane_segments_distance) if is_ingress else last_wp.next(lane_segments_distance)
            
            if len(next_wps) == 0:
                break
            
            next_wp = next_wps[0]
            posRelX = next_wp.transform.location.x - last_pos_x
            posRelY = -next_wp.transform.location.y - last_pos_y # convert y carla into ros frame

            TrafficLightsSensor.add_node(generic_lane_ingress, posRelX, posRelY)
            
            last_pos_x = next_wp.transform.location.x
            last_pos_y = -next_wp.transform.location.y # convert y carla into ros frame
            last_wp = next_wp
            
        return generic_lane_ingress
      
    @staticmethod
    def create_traffic_light_affected_lane_waypoints(traffic_light, junctionPosX, junctionPosY):
        waypoints = TrafficLightsSensor.get_waypoints_from_traffic_light(traffic_light)
        waypoint = waypoints[0]
        print("waypoint road id: ", waypoint.road_id, ", section id: ", waypoint.section_id)
        
        # create ingress line for
        generic_lane_ingress = GenericLane()
        generic_lane_ingress.lane_id.value = waypoint.road_id   
        generic_lane_ingress._lane_attributes.lane_type.choice = TrafficLightsSensor.convert_lane_type(waypoint.lane_type)

        # build the bitstring for ingress line: 128 encodes ingress and 192 encodes egress in big endian format
        generic_lane_ingress.lane_attributes.directional_use.value.append(128)
        generic_lane_ingress.lane_attributes.directional_use.bits_unused = 6
        
        # lane consists of a nodelist of 2 nodes
        generic_lane_ingress.node_list = NodeListXY()
        generic_lane_ingress.node_list.choice = NodeListXY.CHOICE_NODES

        pos_abs_x = waypoint.transform.location.x
        pos_abs_y = -waypoint.transform.location.y # convert y carla into ros frame
        
        pos_rel_junction_x = pos_abs_x - junctionPosX
        pos_rel_junction_y = pos_abs_y - junctionPosY

        TrafficLightsSensor.add_node(generic_lane_ingress, pos_rel_junction_x, pos_rel_junction_y)
            
        last_pos_x = pos_abs_x
        last_pos_y = pos_abs_y
        
        # create an egress/ingress line with a given length
        for i in range(len(waypoints)):
            if i <= 0: 
                i = 1
                
            next_wp = waypoints[i]
            posRelX = next_wp.transform.location.x - last_pos_x
            posRelY = -next_wp.transform.location.y - last_pos_y # convert y carla into ros frame

            TrafficLightsSensor.add_node(generic_lane_ingress, posRelX, posRelY)
            
            last_pos_x = next_wp.transform.location.x
            last_pos_y = -next_wp.transform.location.y # convert y carla into ros frame
            
        return generic_lane_ingress              
       
        
    @staticmethod
    def convert_carla_location_to_ros_vector3(location):
        return np.array([location.x, -location.y, location.z])

    @staticmethod
    def calculate_junction_mean(junction_waypoint_tuples):
        position = np.array([0.0, 0.0, 0.0])
        waypoint_count = 0
        
        # set the lat/lon coordinates of junction as mean of correpsonding traffic light positions
        for waypoint_tuple in junction_waypoint_tuples:
            wp1, wp2 = waypoint_tuple
            
            position = position + TrafficLightsSensor.convert_carla_location_to_ros_vector3(wp1.transform.location)
            position = position + TrafficLightsSensor.convert_carla_location_to_ros_vector3(wp2.transform.location)

            waypoint_count += 2
        
        if waypoint_count > 0:
            position = position / waypoint_count
 
        return (position[0], position[1], position[2])
    
    
    @staticmethod
    def create_etsi_mapem_message(projection_string, junctions, traffic_light_actors, lane_segments_count = 10, lane_segments_distance = 1.0):

        # create MAPEM data
        mapem = MAPEM()
        mapem.map.msg_issue_revision.value = 0
        
        for junctionKey in junctions:
            junctionContainer = junctions[junctionKey]
            junction = junctionContainer['junction_object']
            traffic_lights = junctionContainer['traffic_lights'].values()
            
            # Create intersection geometry
            intersecion_geometry = IntersectionGeometry()
            intersecion_geometry.id.id.value = junction.id
            intersecion_geometry.ref_point.elevation_is_present = True
        
            # set the lat/lon coordinates of junction as mean of correpsonding traffic light positions
            junctionPosX, junctionPosY, junctionPosZ = TrafficLightsSensor.calculate_junction_mean(junctionContainer['waypoints_tuple'])
            
            lat, lon = TrafficLightsSensor.carla_to_latlon(projection_string, junctionPosX, junctionPosY)
            
            intersecion_geometry.ref_point.lat.value = (int)(lat * 10 ** 7)
            intersecion_geometry.ref_point.lon.value = (int)(lon * 10 ** 7)
            intersecion_geometry.ref_point.elevation.value = (int)(junctionPosZ * 10 ** 1)
                
            for traffic_light in traffic_lights:
                pass
                #helper_lane = TrafficLightsSensor.create_traffic_light_affected_lane_waypoints(traffic_light, junctionPosX, junctionPosY)    
                #intersecion_geometry.lane_set.array.append(helper_lane)
                
            # create ingress and egress into and out of the junction
            for waypointTuple in junctionContainer['waypoints_tuple']:
                
                # create GenericLane
                wp1, wp2 = waypointTuple
                #print("waypoint with road_id: ", wp1.road_id)
                
                # create ingress line for first waypoint of the tuple which lead into the junction
                generic_lane_ingress = TrafficLightsSensor.create_junction_lane(True, wp1, junctionPosX, junctionPosY, lane_segments_count, lane_segments_distance)
                intersecion_geometry.lane_set.array.append(generic_lane_ingress)
                
                # create mapem signal group (traffic light) connection to spatem:
                (traffic_waypoint, traffic_light) = TrafficLightsSensor.get_affected_traffic_light_waypoint(traffic_light_actors, wp1.road_id)
                
                if traffic_waypoint != None and traffic_light != None:
                    connection = Connection()
                    
                    connection.signal_group_is_present = True
                    connection.signal_group.value = traffic_light.id
                    
                    generic_lane_ingress.connects_to_is_present = True
                    generic_lane_ingress.connects_to.array.append(connection)
                    
                # create egress line for
                generic_lane_egress = TrafficLightsSensor.create_junction_lane(False, wp2, junctionPosX, junctionPosY, lane_segments_count, lane_segments_distance)
                intersecion_geometry.lane_set.array.append(generic_lane_egress)
                    
            mapem.map.intersections_is_present = True
            mapem.map.intersections.array.append(intersecion_geometry)
            
        return mapem
    
    def debug_publish_traffic_information(self, traffic_light_actors):
        marker_array = MarkerArray()
        marker_id = 0
        
        
        current_time = self.node.get_clock().now().to_msg()
        
        for traffic_light_actor in traffic_light_actors:
            stop_waypoints = traffic_light_actor.carla_actor.get_stop_waypoints()
            road_ids = {}
            
            print("Number of stop points:", len(stop_waypoints))
            for stop_waypoint in stop_waypoints:
                #if stop_waypoint.road_id in road_ids:
                #    continue
                
                #road_ids[stop_waypoint.road_id] = 1

                next_waypoint = stop_waypoint
                
                for i in range(50):
                
                    # Create marker for this traffic light trigger box
                    marker = Marker()
                    marker.header.frame_id = "carla_map"  # Adjust if using a different frame
                    marker.header.stamp = current_time
                    #marker.ns = "traffic_light_stop_waypoints"
                    marker.id = marker_id
                    marker_id += 1
                    
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    
                    # Set marker position
                    marker.pose.position.x = next_waypoint.transform.location.x
                    marker.pose.position.y = -next_waypoint.transform.location.y
                    
                    marker.pose.orientation.w = 1.0 
                    #print("Position: ", marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
                    
                    #print("Jucntion: ", stop_waypoint.is_junction)

                    # Set marker scale (use the extent from trigger box)
                    marker.scale.x = 1.0
                    marker.scale.y = 1.0
                    marker.scale.z = 1.0
                    
                    # Set marker color based on traffic light state
                    if next_waypoint.is_junction:
                        junction = next_waypoint.get_junction()
                        
                        print(" Road id: ", stop_waypoint.road_id, ", junction id: ", junction.id, "segment id: ", stop_waypoint.section_id)
                        found = False
                        
                        for wp1, wp2 in junction.get_waypoints(LaneType.Driving):
                            if wp1.road_id == stop_waypoint.road_id or wp1.road_id:
                                found = True
                                break
                            
                        if found == True:
                            marker.color.r = 0.0
                            marker.color.g = 0.0
                            marker.color.b = 1.0
                            
                            marker.scale.x = 1.6
                            marker.scale.y = 1.6
                            marker.scale.z = 1.6
                        else:
                            marker.color.r = 1.0
                            marker.color.g = 0.0
                            marker.color.b = 0.0
                    else:
                        marker.color.r = 0.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                        
                    marker.color.a = 1.0  # Full opacity
                    marker.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()  # Short lifetime until next update
                    
                    # Add to marker array
                    marker_array.markers.append(marker)
                    
                    if next_waypoint.is_junction:
                        break
                    
                    next_waypoint = next_waypoint.next(1)[0]
                    
    
            
            if False:
                for tl in stop_waypoints:
                    # Get traffic light transform
                    tl_transform = tl.get_info().transform
                    tl_location = tl_transform.position
                    tl_rotation = tl_transform.orientation
                    print("Create marker before")
                    # Get trigger box (bounding box) - in CARLA this is the 'trigger_volume'
                    trigger_box = tl.get_info().trigger_volume
                    if trigger_box is None:
                        print("No trigger volume for traffic light ")
                        continue
                    print("Create marker")
                    extent = trigger_box.size
                    box_transform_location = trigger_box.center
                    
                    # Create marker for this traffic light trigger box
                    marker = Marker()
                    marker.header.frame_id = "carla_map"  # Adjust if using a different frame
                    marker.header.stamp = current_time
                    marker.ns = "traffic_light_triggers"
                    marker.id = marker_id
                    marker_id += 1
                    
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    
                    # Set marker position
                    marker.pose.position.x = tl_location.x + box_transform_location.x
                    marker.pose.position.y = tl_location.y + box_transform_location.y
                    marker.pose.position.z = tl_location.z + box_transform_location.z

                    
                    # Set marker scale (use the extent from trigger box)
                    marker.scale.x = extent.x * 2.0  # CARLA uses half-dimensions for extent
                    marker.scale.y = extent.y * 2.0
                    marker.scale.z = extent.z * 2.0
                    
                    # Set marker color based on traffic light state
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                        
                    marker.color.a = 0.3  # Semi-transparent
                    marker.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()  # Short lifetime until next update
                    
                    # Add to marker array
                    marker_array.markers.append(marker)
            
        # Publish the marker array
        self.marker_publisher.publish(marker_array)


    
    @staticmethod
    def create_etsi_spatem_message(junctions):
        spatem = SPATEM()
        spatem.spat.name_is_present = True
        spatem.spat.name.value = "Carla traffic light status"
        
        for junctionKey in junctions:
            junctionContainer = junctions[junctionKey]
            junction = junctionContainer['junction_object']
            
            # Create intersection geometry
            intersection_state = IntersectionState()
            intersection_state.id.id.value = junction.id
            
            for traffic_light in junctionContainer['traffic_lights'].values():
                print("Spatem withn raffic light id: ", traffic_light.id)
                # Movement State
                movement_state = MovementState()
                movement_state.signal_group.value = traffic_light.id
                
                # Movement event
                movement_event = MovementEvent()
                movement_event.event_state.value = TrafficLightsSensor.convert_traffic_light_state(traffic_light.state)
                
                # fill arrays
                movement_state.state_time_speed.array.append(movement_event)
                intersection_state.states.array.append(movement_state)
                
            spatem.spat.intersections.array.append(intersection_state)
            
        return spatem

    def update(self, frame, timestamp):
        """
        Get the state of all known traffic lights
        """
        traffic_light_status = CarlaTrafficLightStatusList()
        traffic_light_actors = []
        for actor_id in self.actor_list:
            actor = self.actor_list[actor_id]
            if isinstance(actor, TrafficLight):
                traffic_light_actors.append(actor)
                traffic_light_status.traffic_lights.append(actor.get_status())

        if traffic_light_actors != self.traffic_light_actors:
            self.traffic_light_actors = traffic_light_actors
            traffic_light_info_list = CarlaTrafficLightInfoList()
            for traffic_light in traffic_light_actors:
                traffic_light_info_list.traffic_lights.append(traffic_light.get_info())
            self.traffic_lights_info_publisher.publish(traffic_light_info_list)

        if traffic_light_status != self.traffic_light_status:
            self.traffic_light_status = traffic_light_status
            self.traffic_lights_status_publisher.publish(traffic_light_status)

        
        # create Etsi data structure
        self.publish_etsi_messages(traffic_light_actors)
        self.debug_publish_traffic_information(traffic_light_actors)
        
        
        
