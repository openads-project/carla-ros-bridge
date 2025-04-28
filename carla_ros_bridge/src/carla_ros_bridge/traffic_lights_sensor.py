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
        self.publish_etsi_messages = node.parameters['publish_etsi_messages']
        self.waypoints_search_distance = node.parameters['waypoints_search_distance']
        self.lane_waypoints_count = node.parameters['lane_waypoints_count']
        self.taffic_light_junction_max_search_count = node.parameters['taffic_light_junction_max_search_count']
        self.debug_traffic_light_information = node.parameters['debug_traffic_light_information']
        self.integrate_junctions_without_traffic_lights = node.parameters['integrate_junctions_without_traffic_lights']
        self.traffic_light_junction_search_ignored_ids = node.parameters['traffic_light_junction_search_ignored_ids']
        
        traffic_light_actors = self.get_traffic_light_actors()
        self.initialize_junctions(traffic_light_actors)       

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
        
        if self.publish_etsi_messages:
            # spatem publisher calllback
            timer_period = node.parameters['publisher_spatem_timer_period'] # seconds
            self.timer_mapem = node.create_timer(timer_period, self.publish_etsi_mapem_message)
            
            # mapem publisher calllback
            timer_period = node.parameters['publisher_mapem_timer_period'] # seconds
            self.timer_spatem = node.create_timer(timer_period, self.publish_etsi_spatem_message)
            
            if self.debug_traffic_light_information == True:
                # publish debug information
                timer_period = node.parameters['publisher_debug_traffic_light_information_timer_period'] # seconds
                self.timer_traffic_lights_debug = node.create_timer(timer_period, self.debug_publish_traffic_information)

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
    
    def get_traffic_light_actors(self):
        """
        Returns an array of Carla actors of the derived type TrafficLight
        :return array of all TrafficLight Actor objects inside the Carla world
        :rtype array(carla.TrafficLight)
        """
        traffic_light_actors = []
        for actor_id in self.actor_list:
            actor = self.actor_list[actor_id]
            if isinstance(actor, TrafficLight):
                traffic_light_actors.append(actor)
                
        return traffic_light_actors

    def get_junction(self, junction_id):
        """
        Returns a junction with the given id. The id of the carla.Juction corresponds to the id of the junction in the OpenDrive file.
        :return Carla junction from the Carla world object
        :rtype carla.Junction
        """
        return self.junctions[junction_id]['junction_object']
        
    def get_junction_traffic_lights(self, junction_id):
        """
        Returns all traffic lights that belong to a junction with a given id
        :return Traffic lights of a junction
        :rtype array(carla.Junction)
        """
        return self.junctions[junction_id]['traffic_lights'].values()
        
    def get_junction_waypoints(self, junction_id):
        """
        Returns a list of waypoint tuples (a waypoint tuple describes the start- and end waypoint of a lane inside a junction) and the corresponding traffic light if available
        :return a tuple with the fomat (waypoint ingoing, waypoint outgoing, traffic light for ingoing waypoint if available)
        :rtype tuple(carla.Waypoint, carla.Waypoint, carla.TrafficLight)
        """
        return self.junctions[junction_id]['waypoints_tuple']
        
    def get_junction_position(self, junction_id):
        return self.junctions[junction_id]['position']        
        
    def set_junction(self, junction_id, value):
        self.junctions[junction_id]['junction_object'] = value
        
    def set_junction_traffic_lights(self, junction_id, value):
        self.junctions[junction_id]['traffic_lights'] = value
        
    def set_junction_waypoints(self, junction_id, value):
        self.junctions[junction_id]['waypoints_tuple'] = value
        
    def set_junction_position(self, junction_id, value):
        self.junctions[junction_id]['position'] = value
    
    @staticmethod
    def set_etsi_lat_lon_junction(etsi_junction, lat, lon, z):
        etsi_junction.ref_point.lat.value = (int)(lat * 10 ** 7)
        etsi_junction.ref_point.lon.value = (int)(lon * 10 ** 7)
        etsi_junction.ref_point.elevation.value = (int)(z * 10 ** 1)  
    
    @staticmethod
    def convert_carla_location_to_ros_vector3(location):
        return np.array([location.x, -location.y, location.z])

    
    @staticmethod
    def carla_to_latlon(projection_string, carla_x, carla_y):
        """
        Convert CARLA coordinates to latitude/longitude using PyProj and a reference point.
        
        Args:
            carla_x, carla_y: CARLA coordinates to convert
            projection_string: Projection string whichn is used to convert Carla coordinates to lat/lon coordinates
        
        Returns:
            latitude, longitude: WGS84 coordinates
        """
        proj_xodr = pyproj.Proj(projparams=projection_string)
        lon, lat = proj_xodr(carla_x, carla_y, inverse=True)
        
        return lat, lon

    @staticmethod
    def convert_lane_type(carla_lane_type : LaneType):
        """
        Convert a CARLA LaneType into an Etsi LaneType
        
        Args:
            carla_lane_type: LaneType as Carla Type
        
        Returns:
            LaneType as Etsi type
        """
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
        """
        Convert the type CarlaTrafficLightStatus into the corresponding Etsi type
        
        Args:
            state: Carla status as type CarlaTrafficLightStatus
        
        Returns:
            LaneType as Etsi type
        """
        state_dictionary = {
            CarlaTrafficLightStatus.RED: MovementPhaseState.STOP_THEN_PROCEED,
            CarlaTrafficLightStatus.YELLOW: MovementPhaseState.PRE_MOVEMENT,
            CarlaTrafficLightStatus.GREEN: MovementPhaseState.PERMISSIVE_MOVEMENT_ALLOWED,
            CarlaTrafficLightStatus.OFF: MovementPhaseState.DARK,
            CarlaTrafficLightStatus.UNKNOWN: MovementPhaseState.DARK    
        }
        
        return state_dictionary[state]

    @staticmethod
    def add_lane_node(lane, position):
        node = NodeXY()
        node.delta.node_xy1.x.value = (int)(position[0] * 100)
        node.delta.node_xy1.y.value = (int)(position[1] * 100)
        lane.node_list.nodes.array.append(node)
    
    def initialize_junctions(self, traffic_lights):
        self.junctions = {}
        map = self.node.carla_world.get_map()
        all_waypoints = map.generate_waypoints(self.waypoints_search_distance)
        
        # iterate through all waypoints inside the carla map and save all found junctions
        for waypoint in all_waypoints:
            if waypoint.is_junction:
                # build the junction data structure with all it's traffic lights and in- and outgoing driving lanes
                junction_id = waypoint.junction_id
                
                if junction_id not in self.junctions:
                    
                    # get all waypoint tuples for a given junction
                    # a tuple represents a driving line from the entrance to a junction ingress -first tuple element) 
                    # to an exit (egress - second tuple element)
                    # an ingress lane can have an attached traffic light 
                    junction_object = waypoint.get_junction()
                    waypoint_tuples = junction_object.get_waypoints(LaneType.Driving)
                    waypoint_tuples_traffic_lights = []
                    junction_traffic_lights = {}
                    junction_position = TrafficLightsSensor.calculate_junction_mean(waypoint_tuples)
                    
                    # connect traffic light with corresponding ingress lane waypoint if available
                    for waypointTuple in waypoint_tuples:
                        wp1, wp2 = waypointTuple
                        
                        (waypoint, traffic_light) = self.get_affected_traffic_light_waypoint(traffic_lights, wp1.road_id)
                        
                        if (waypoint, traffic_light) != (None, None):
                            junction_traffic_lights[traffic_light.id] = traffic_light
                            
                        waypoint_tuples_traffic_lights.append([wp1, wp2, traffic_light])
                        
                    # fill junction data structure
                    if self.integrate_junctions_without_traffic_lights or len(junction_traffic_lights) > 0:
                        self.junctions[junction_id] = {}
                        self.set_junction(junction_id, junction_object)
                        self.set_junction_traffic_lights(junction_id, junction_traffic_lights)
                        self.set_junction_waypoints(junction_id, waypoint_tuples_traffic_lights)
                        self.set_junction_position(junction_id, junction_position)
                                        
    
    @staticmethod
    def get_waypoints_from_traffic_light(traffic_light):
        """
        Returns a single waypoint for each lane affected by the given traffic light.
        The given implementation performs a brute force search for all affected lanes within the Carla c++ implementation.
        For each traffic light, all waypoints within the corresponding traffic light trigger box are searched. If the lane associated with the waypoint is not in the output array, it is added.
        
        :return List of waypoints, each corresponding to a different lane.
        :rtype list(carla.Waypoint)
        """
        return traffic_light.get_stop_waypoints()

    def get_affected_traffic_light_waypoint(self, traffic_lights, road_id):
        """
        Given an id of a road inside a junction, this method returns the corresponding traffic light for the stop line leading into the junction to the given road id.
        :param traffic_lights: all traffic light actors from the carla world
        :type traffic_lights: array(carla.TrafficLight)
        :param road_id: OpenDRIVE road's id
        :type road_id: int
        :return a tuple of the first edge intersection waypoint which leads into the intersection and is part of the road with the given road_id and the Traffic Light
        :rtype tuple(carla.Waypoint, carla.TrafficLight) 
        """

        for traffic_light in traffic_lights:
            stop_waypoints = TrafficLightsSensor.get_waypoints_from_traffic_light(traffic_light.carla_actor)
            
            for stop_waypoint in stop_waypoints:
                waypoint = stop_waypoint
                
                for i in range(self.taffic_light_junction_max_search_count):
                    if waypoint.is_junction:
                        # a junction has been found
                        # ignore junctions from the blacklist
                        ignore_junction = False
                        
                        for ignored_junction_id in self.traffic_light_junction_search_ignored_ids:
                            if waypoint.get_junction().id == ignored_junction_id:
                                ignore_junction = True
                                break
                        
                        if ignore_junction == False and road_id == waypoint.road_id:
                            return (waypoint, traffic_light.carla_actor)
                    
                    # Get the next waypoint in the list
                    waypoint = waypoint.next(1.0)[0]
                    
                    # this prevents the the lane to be counted multiple times for the same junction
                    for test_waypoint in stop_waypoints:
                        if test_waypoint.id == waypoint.id:
                            break
                
        return (None, None)
                    
        
    def create_junction_lane(self, is_ingress, waypoint, junction_position):
        """
        Creates an Ingress or Egress lane.
        The lane is part of an Etsi Mapem message.
        :param is_ingress: The lane is an Ingress lane (true, enters the intersection) or an Egress lane (false, exits the intersection)
        :type is_ingress: bool
        :param waypoint: the waypoint corresponding to the lane at the start/end of the intersection
        :type waypoint: carla.Waypoint
        :param junction_position: position of the junction
        :type junction_position: numpy.array(3)
        :return Etsi Mapem lane (Ingress or Egress)
        :rtype GenericLane
        """

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

        pos_abs = TrafficLightsSensor.convert_carla_location_to_ros_vector3(waypoint.transform.location)        
        pos_rel_junction = pos_abs - junction_position
        
        TrafficLightsSensor.add_lane_node(generic_lane_ingress, pos_rel_junction)
        
        last_wp = waypoint
        last_pos = pos_abs
        
        # create an egress/ingress line with a given length
        for i in range(self.lane_waypoints_count):
            next_wps = last_wp.previous(self.waypoints_search_distance) if is_ingress else last_wp.next(self.waypoints_search_distance)
            
            if len(next_wps) == 0:
                break
            
            next_wp = next_wps[0]
            next_wp_position = TrafficLightsSensor.convert_carla_location_to_ros_vector3(next_wp.transform.location)
            
            pos_rel = next_wp_position - last_pos
            TrafficLightsSensor.add_lane_node(generic_lane_ingress, pos_rel)
            last_pos = next_wp_position
            last_wp = next_wp
            
        return generic_lane_ingress
    
    @staticmethod
    def calculate_junction_mean(junction_waypoint_tuples):
        """
        Calculates the position of a junction by using the mean position of all edge waypoints.
        :param junction_waypoint_tuples: all driving lane waypoints from the edge of the junction (ingoing and outgoing)
        :return mean position of all positions from the dribing lane intersetion tuples
        :rtype numpy.array(3)
        """
        position = np.array([0.0, 0.0, 0.0])
        waypoint_count = 0
        
        # set the lat/lon coordinates of junction as mean of correpsonding traffic light positions
        for waypoint_tuple in junction_waypoint_tuples:
            position = position + TrafficLightsSensor.convert_carla_location_to_ros_vector3(waypoint_tuple[0].transform.location)
            position = position + TrafficLightsSensor.convert_carla_location_to_ros_vector3(waypoint_tuple[1].transform.location)

            waypoint_count += 2
        
        if waypoint_count > 0:
            position = position / waypoint_count
 
        return (position)


    def publish_etsi_mapem_message(self):
        """
        Creates and publishes an Etsi Mapem message
        """
        # create MAPEM data
        mapem = MAPEM()
        mapem.map.msg_issue_revision.value = 0
        
        for junction_id in self.junctions:
            junction = self.get_junction(junction_id)
            junction_position = self.get_junction_position(junction_id)
            waypoint_tuples = self.get_junction_waypoints(junction_id)
            
            # Create intersection geometry
            intersecion_geometry = IntersectionGeometry()
            intersecion_geometry.id.id.value = junction.id
            intersecion_geometry.ref_point.elevation_is_present = True
        
            # set the lat/lon coordinates of junction as mean of correpsonding traffic light positions
            projection_string = self.node.world_info.projection_string
            lat, lon = TrafficLightsSensor.carla_to_latlon(projection_string, junction_position[0], junction_position[1])
            TrafficLightsSensor.set_etsi_lat_lon_junction(intersecion_geometry, lat, lon, junction_position[2])
                
            # create ingress and egress into and out of the junction
            
            for waypointTuple in waypoint_tuples:
                
                # create ingress line for first waypoint of the tuple which lead into the junction
                generic_lane_ingress = self.create_junction_lane(True, waypointTuple[0], junction_position)
                intersecion_geometry.lane_set.array.append(generic_lane_ingress)
                
                # if the ingress lane is affected by a traffic light, connect it to the traffic light
                if waypointTuple[2] != None:
                    traffic_light = waypointTuple[2]
                    connection = Connection()
                    
                    connection.signal_group_is_present = True
                    connection.signal_group.value = traffic_light.id
                    
                    generic_lane_ingress.connects_to_is_present = True
                    generic_lane_ingress.connects_to.array.append(connection)
                    
                # create egress line for
                generic_lane_egress = self.create_junction_lane(False, waypointTuple[1], junction_position)
                intersecion_geometry.lane_set.array.append(generic_lane_egress)
                    
            mapem.map.intersections_is_present = True
            mapem.map.intersections.array.append(intersecion_geometry)
            
        self.etsi_mapem_publisher.publish(mapem)
    
    def debug_publish_traffic_information(self):
        """
        Publishes a debug MarkerArray which visualizes the steps of the method get_affected_traffic_light_waypoint()
        """
        marker_array = MarkerArray()
        marker_id = 0
        
        current_time = self.node.get_clock().now().to_msg()
        
        for traffic_light_actor in self.traffic_light_actors:
            stop_waypoints = traffic_light_actor.carla_actor.get_stop_waypoints()
            
            for stop_waypoint in stop_waypoints:
                next_waypoint = stop_waypoint
                
                for i in range(self.taffic_light_junction_max_search_count):
                
                    # Create marker for this traffic light trigger box
                    marker = Marker()
                    marker.header.frame_id = "carla_map"
                    marker.header.stamp = current_time
                    marker.id = marker_id
                    marker_id += 1
                    
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    
                    # Set marker position
                    marker.pose.position.x = next_waypoint.transform.location.x
                    marker.pose.position.y = -next_waypoint.transform.location.y
                    
                    marker.pose.orientation.w = 1.0 

                    # Set marker scale (use the extent from trigger box)
                    marker.scale.x = 1.0
                    marker.scale.y = 1.0
                    marker.scale.z = 1.0
                    
                    # Set marker color based on traffic light state
                    if next_waypoint.is_junction:
                        junction = next_waypoint.get_junction()
                        
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
                    
        # Publish the marker array
        self.marker_publisher.publish(marker_array)


    def publish_etsi_spatem_message(self):
        """
        Creates and publishes an Etsi Spatem message
        """
        spatem = SPATEM()
        spatem.spat.name_is_present = True
        spatem.spat.name.value = "Carla traffic light status"
        
        for junction_id in self.junctions:
            junction = self.get_junction(junction_id)
            traffic_lights = self.get_junction_traffic_lights(junction_id)
            
            # Create intersection geometry
            intersection_state = IntersectionState()
            intersection_state.id.id.value = junction.id
            
            for traffic_light in traffic_lights:
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
            
        self.etsi_spatem_publisher.publish(spatem)

    def update(self, frame, timestamp):
        """
        Get the state of all known traffic lights
        """
        traffic_light_actors = self.get_traffic_light_actors()
        traffic_light_status = CarlaTrafficLightStatusList()

        for traffic_light in traffic_light_actors:
            traffic_light_status.traffic_lights.append(traffic_light.get_status())

        # publish traffic light info
        if traffic_light_actors != self.traffic_light_actors:
            self.traffic_light_actors = traffic_light_actors
            traffic_light_info_list = CarlaTrafficLightInfoList()
            for traffic_light in traffic_light_actors:
                traffic_light_info_list.traffic_lights.append(traffic_light.get_info())
            self.traffic_lights_info_publisher.publish(traffic_light_info_list)

        # publish traffic light status
        if traffic_light_status != self.traffic_light_status:
            self.traffic_light_status = traffic_light_status
            self.traffic_lights_status_publisher.publish(traffic_light_status)
        
        
        
        
        
