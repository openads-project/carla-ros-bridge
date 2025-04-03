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

from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_ros_bridge.pseudo_actor import PseudoActor
from carla_ros_bridge.traffic import TrafficLight
import tf2_ros

from carla_msgs.msg import (
    CarlaTrafficLightStatusList,
    CarlaTrafficLightInfoList
)

from carla_msgs.msg import CarlaTrafficLightStatus, CarlaTrafficLightInfo

from carla.libcarla import LaneType
from carla.libcarla import LaneChange

from etsi_its_mapem_ts_msgs.msg import MAPEM
from etsi_its_mapem_ts_msgs.msg import IntersectionGeometry
from etsi_its_mapem_ts_msgs.msg import GenericLane
from etsi_its_mapem_ts_msgs.msg import Connection
from etsi_its_mapem_ts_msgs.msg import NodeListXY
from etsi_its_mapem_ts_msgs.msg import NodeXY
from etsi_its_mapem_ts_msgs.msg import LaneTypeAttributes
from etsi_its_mapem_ts_msgs.msg import Elevation

from etsi_its_spatem_ts_msgs.msg import IntersectionState
from etsi_its_spatem_ts_msgs.msg import MovementState
from etsi_its_spatem_ts_msgs.msg import MovementPhaseState
from etsi_its_spatem_ts_msgs.msg import MovementEvent


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
    
    def get_junctions(self, traffic_light_actors):
        junctions = {}
        
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
                
                if junction_waypoint != None:
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
                    
                    
        return junctions
    
    @staticmethod
    def get_waypoints_from_traffic_light(traffic_light):
        waypoints = traffic_light.carla_actor.get_affected_lane_waypoints()
        
        if (len(waypoints) == 0):
            waypoints = traffic_light.carla_actor.get_stop_waypoints()
        
        return waypoints
    
    
    @staticmethod
    def get_affected_traffic_light_waypoint(traffic_lights, road_id):
        
        for traffic_light in traffic_lights:
            waypoints = TrafficLightsSensor.get_waypoints_from_traffic_light(traffic_light)
            waypoint = waypoints[0]
            
            for waypoint in waypoints:
                if waypoint.road_id == road_id:
                    return (waypoint, traffic_light)
                
        return (None, None)
                    
    def publish_etsi_messages(self, traffic_light_actors):
        junctions = self.get_junctions(traffic_light_actors)
        
        mapem = TrafficLightsSensor.create_etsi_mapem_message(self.node.world_info.projection_string, junctions, 10, 1.0)
        self.etsi_mapem_publisher.publish(mapem)
        
        spatem = TrafficLightsSensor.create_etsi_spatem_message(junctions)
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
            print("waypoint road id: ", next_wp.road_id, ", section id: ", next_wp.section_id)
            TrafficLightsSensor.add_node(generic_lane_ingress, posRelX, posRelY)
            
            last_pos_x = next_wp.transform.location.x
            last_pos_y = -next_wp.transform.location.y # convert y carla into ros frame
            
        return generic_lane_ingress              
       
        
    @staticmethod
    def calculate_junction_mean(traffic_lights):
        # set the lat/lon coordinates of junction as mean of correpsonding traffic light positions
        junctionPosX = 0
        junctionPosY = 0
        junctionPosZ = 0
        junctionCount = 0
        
        for traffic_light in traffic_lights:
                waypoints = TrafficLightsSensor.get_waypoints_from_traffic_light(traffic_light)
                wp = waypoints[0]
                
                junctionPosX += wp.transform.location.x
                junctionPosY += -wp.transform.location.y # convert y carla into ros frame
                junctionPosZ += +wp.transform.location.z
                junctionCount += 1
        
        if junctionCount > 0:
            junctionPosX = junctionPosX / junctionCount
            junctionPosY = junctionPosY / junctionCount
            junctionPosZ = junctionPosZ / junctionCount    
            
        return (junctionPosX, junctionPosY, junctionPosZ)    
    
    
    @staticmethod
    def create_etsi_mapem_message(projection_string, junctions, lane_segments_count = 10, lane_segments_distance = 1.0):

        # create MAPEM data
        mapem = MAPEM()
        mapem.map.msg_issue_revision.value = 0
        
        for junctionKey in junctions:
            print("_______")
            print("")
            print ("Junction with id:", junctionKey)
            
            junctionContainer = junctions[junctionKey]
            junction = junctionContainer['junction_object']
            traffic_lights = junctionContainer['traffic_lights'].values()
            
            # Create intersection geometry
            intersecion_geometry = IntersectionGeometry()
            intersecion_geometry.id.id.value = junction.id
            intersecion_geometry.ref_point.elevation_is_present = True
        
            # set the lat/lon coordinates of junction as mean of correpsonding traffic light positions
            junctionPosX, junctionPosY, junctionPosZ = TrafficLightsSensor.calculate_junction_mean(traffic_lights)
            
            lat, lon = TrafficLightsSensor.carla_to_latlon(projection_string, junctionPosX, junctionPosY)
            
            intersecion_geometry.ref_point.lat.value = (int)(lat * 10 ** 7)
            intersecion_geometry.ref_point.lon.value = (int)(lon * 10 ** 7)
            intersecion_geometry.ref_point.elevation.value = (int)(junctionPosZ * 10 ** 1)
                
            for traffic_light in traffic_lights:
                helper_lane = TrafficLightsSensor.create_traffic_light_affected_lane_waypoints(traffic_light, junctionPosX, junctionPosY)    
                intersecion_geometry.lane_set.array.append(helper_lane)
                
            # create ingress and egress into and out of the junction
            for waypointTuple in junctionContainer['waypoints_tuple']:
                
                # create GenericLane
                wp1, wp2 = waypointTuple
                print("waypoint with road_id: ", wp1.road_id)
                
                # create ingress line for first waypoint of the tuple which lead into the junction
                generic_lane_ingress = TrafficLightsSensor.create_junction_lane(True, wp1, junctionPosX, junctionPosY, lane_segments_count, lane_segments_distance)
                intersecion_geometry.lane_set.array.append(generic_lane_ingress)
                
                # create mapem signal group (traffic light) connection to spatem:
                (traffic_waypoint, traffic_light) = TrafficLightsSensor.get_affected_traffic_light_waypoint(traffic_lights, wp1.road_id)
                
                if traffic_waypoint != None and traffic_light != None:
                    connection = Connection()
                    
                    connection.signal_group_is_present = True
                    connection.signal_group.value = traffic_light.uid
                    
                    generic_lane_ingress.connects_to_is_present = True
                    generic_lane_ingress.connects_to.array.append(connection)
                
                # create egress line for
                generic_lane_egress = TrafficLightsSensor.create_junction_lane(False, wp2, junctionPosX, junctionPosY, lane_segments_count, lane_segments_distance)
                intersecion_geometry.lane_set.array.append(generic_lane_egress)
                    
            mapem.map.intersections_is_present = True
            mapem.map.intersections.array.append(intersecion_geometry)
            
        return mapem
    
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
                status = traffic_light.get_status()
                
                # Movement State
                movement_state = MovementState()
                movement_state.signal_group.value = traffic_light.uid
                
                # Movement event
                movement_event = MovementEvent()
                movement_event.event_state.value = TrafficLightsSensor.convert_traffic_light_state(status.state)
                
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
        
