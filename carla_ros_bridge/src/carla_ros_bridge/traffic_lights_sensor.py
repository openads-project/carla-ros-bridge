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
import carla_ros_bridge.bridge
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
        
        # Set up Buffer and TransformListener to lookup transforms between frames
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, node, spin_thread=False)
        

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

    @staticmethod
    def transform_coordinates_utm_to_latlon(x, y):
        offset_30_x = 833978.5569194595
        offset_31_x = 166021.44308054057 
        offset_N_y = 0
        offset_S_y = 10000000

        # Define UTM projection using the specified zone
        if x >= 0:
            zone = 31
            x += offset_31_x
        else:
            zone = 30
            x += offset_30_x

        if y >= 0:
            northp = True
            y += offset_N_y
        else:
            northp = False
            y += offset_S_y 
        
        if northp:
            utm_proj = pyproj.Proj(proj='utm',zone=zone,ellps='WGS84', preserve_units=False)
        else:
            utm_proj = pyproj.Proj(proj='utm',zone=zone, south=True, ellps='WGS84', preserve_units=False)
        
        # Define WGS84 projection (latitude, longitude)
        latlon_proj = pyproj.Proj(proj='latlong', datum='WGS84')
        
        # Perform the transformation from UTM to Latitude/Longitude
        longitude, latitude = pyproj.transform(utm_proj, latlon_proj, x, y)
        
        return latitude, longitude


    @staticmethod
    def get_lane_direction_vector(waypoint):
        # Get the transform of the waypoint (position and rotation)
        transform = waypoint.transform

        # Extract the yaw (rotation around the Z-axis) to get the heading of the waypoint
        yaw = transform.rotation.yaw

        # Convert yaw to radians and calculate the direction vector
        radian = math.radians(yaw)

        # The direction vector in the xy-plane (forward direction of the car)
        x, y = math.cos(radian), -math.sin(radian)

        return x, y

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
    
    @staticmethod
    def get_junctions(traffic_light_actors):
        junctions = {}
        
        for actor in traffic_light_actors:
            if isinstance(actor, TrafficLight):
                traffic_light = actor
                waypoints_traffic_light = traffic_light.carla_actor.get_affected_lane_waypoints()
                
                #for waypoint in waypoints:
                waypoint = waypoints_traffic_light[0]
                
                if waypoint.is_junction:
                    junction_id = waypoint.junction_id

                    # Create new junction entry if it doesn't exist
                    if junction_id not in junctions:
                        # Try to get the actual junction object
                        junction_object = waypoint.get_junction()
                        
                        junctions[junction_id] = {
                            'junction_object': junction_object,
                            'traffic_lights': {},
                            'waypoints_tuple': []
                        }
                        
                        waypoints = junction_object.get_waypoints(LaneType.Driving)
                        
                        for waypoint in waypoints:
                            junctions[junction_id]['waypoints_tuple'].append(waypoint)
                        
                    junctions[junction_id]['traffic_lights'][traffic_light.uid] = traffic_light
                    
        return junctions
        
    def publish_etsi_messages(self, traffic_light_actors):
        junctions = TrafficLightsSensor.get_junctions(traffic_light_actors)
        
        print("Try to publish mapem")
        mapem = TrafficLightsSensor.create_etsi_mapem_message(junctions, 10, 1.0)
        self.etsi_mapem_publisher.publish(mapem)
        
        print ("Try to publish spatem")
        spatem = TrafficLightsSensor.create_tesi_spatem_message(junctions)
        self.etsi_spatem_publisher.publish(spatem)
        
    @staticmethod
    def create_etsi_mapem_message(junctions, lane_segments_count = 10, lane_segments_distance = 1.0):
        # create MAPEM data
        mapem = MAPEM()
        mapem.map.msg_issue_revision.value = 0
        
        for junctionKey in junctions:
            junctionContainer = junctions[junctionKey]
            junction = junctionContainer['junction_object']
            
            # Create intersection geometry
            intersecion_geometry = IntersectionGeometry()
            intersecion_geometry.id.id.value = junction.id
            intersecion_geometry.ref_point.elevation_is_present = True
        
            # set the lat/lon coordinates of junction as mean of correpsonding traffic light positions
            junctionPosX = 0
            junctionPosY = 0
            junctionPosZ = 0
            junctionCount = 0
            
            for traffic_light in junctionContainer['traffic_lights'].values():
                    waypoints = traffic_light.carla_actor.get_affected_lane_waypoints()
                    wp = waypoints[0]
                    
                    junctionPosX += wp.transform.location.x
                    junctionPosY += -wp.transform.location.y
                    junctionPosZ += +wp.transform.location.z
                    junctionCount += 1
            
            if junctionCount > 0:
                junctionPosX = junctionPosX / junctionCount
                junctionPosY = junctionPosY / junctionCount
                junctionPosZ = junctionPosZ / junctionCount
                
                lat, lon = TrafficLightsSensor.transform_coordinates_utm_to_latlon(junctionPosX, junctionPosY)
                intersecion_geometry.ref_point.lon.value = (int)(lon * 10 ** 7)
                intersecion_geometry.ref_point.lat.value = (int)(lat * 10 ** 7)
                intersecion_geometry.ref_point.elevation.value = (int)(junctionPosZ * 10 ** 1)
            
            # create traffic lights in a virtual lane
            for traffic_light in junctionContainer['traffic_lights'].values():
                info = traffic_light.get_info()
                
                generic_lane = GenericLane()
                generic_lane.lane_attributes.directional_use.value.append(192)
                generic_lane.lane_attributes.directional_use.bits_unused = 6
                
                connection = Connection()
                    
                connection.signal_group_is_present = True
                connection.signal_group.value = traffic_light.uid
                
                generic_lane.connects_to_is_present = True
                generic_lane.connects_to.array.append(connection)
                
                # lane consists of a nodelist of 2 nodes
                generic_lane.node_list = NodeListXY()
                generic_lane.node_list.choice = NodeListXY.CHOICE_NODES
                
                posAbsX = info.transform.position.x - junctionPosX
                posAbsY = info.transform.position.y - junctionPosY
                
                TrafficLightsSensor.add_node(generic_lane, posAbsX, posAbsY)
                TrafficLightsSensor.add_node(generic_lane, 0, 0)
                
                intersecion_geometry.lane_set.array.append(generic_lane)
                
            # create ingress and egress into and out of the junction
            for waypointTuple in junctionContainer['waypoints_tuple']:
                
                # create GenericLane
                wp1, wp2 = waypointTuple

                # create ingress line for
                generic_lane_ingress = GenericLane()
                generic_lane_ingress.lane_id.value = wp1.road_id   
                generic_lane_ingress._lane_attributes.lane_type.choice = TrafficLightsSensor.convert_lane_type(wp1.lane_type)

                generic_lane_ingress.lane_attributes.directional_use.value.append(128)
                generic_lane_ingress.lane_attributes.directional_use.bits_unused = 6
                
                # lane consists of a nodelist of 2 nodes
                generic_lane_ingress.node_list = NodeListXY()
                generic_lane_ingress.node_list.choice = NodeListXY.CHOICE_NODES

                posAbsX = wp1.transform.location.x
                posAbsY = -wp1.transform.location.y

                TrafficLightsSensor.add_node(generic_lane_ingress, posAbsX - junctionPosX, posAbsY - junctionPosY)
                    
                last_wp = wp1
                lastPosX = posAbsX
                lastPosY = posAbsY
                
                # 
                for i in range(lane_segments_count):
                    next_wps = last_wp.previous(lane_segments_distance)
                    
                    if len(next_wps) == 0:
                        break
                    
                    next_wp = next_wps[0]
                    posRelX = next_wp.transform.location.x - lastPosX
                    posRelY = -next_wp.transform.location.y - lastPosY

                    TrafficLightsSensor.add_node(generic_lane_ingress, posRelX, posRelY)
                    
                    lastPosX = next_wp.transform.location.x
                    lastPosY = -next_wp.transform.location.y
                    last_wp = next_wp
                    
                intersecion_geometry.lane_set.array.append(generic_lane_ingress)
                
                
                # create egress line for
                generic_lane_egress = GenericLane()
                generic_lane_egress.lane_id.value = wp2.road_id   
                generic_lane_egress._lane_attributes.lane_type.choice = TrafficLightsSensor.convert_lane_type(wp2.lane_type)

                generic_lane_egress.lane_attributes.directional_use.value.append(64)
                generic_lane_egress.lane_attributes.directional_use.bits_unused = 6
                
                # lane consists of a nodelist of 2 nodes
                generic_lane_egress.node_list = NodeListXY()
                generic_lane_egress.node_list.choice = NodeListXY.CHOICE_NODES
                
                posAbsX = wp2.transform.location.x
                posAbsY = -wp2.transform.location.y
            
                TrafficLightsSensor.add_node(generic_lane_egress, posAbsX - junctionPosX, posAbsY - junctionPosY)

                last_wp = wp2
                lastPosX = posAbsX
                lastPosY = posAbsY
                
                for i in range(lane_segments_count):
                    next_wps = last_wp.next(lane_segments_distance)
                    if len(next_wps) == 0:
                        break
                    
                    next_wp = next_wps[0]
                    posRelX = next_wp.transform.location.x - lastPosX
                    posRelY = -next_wp.transform.location.y - lastPosY
                    
                    TrafficLightsSensor.add_node(generic_lane_egress, posRelX, posRelY)
                    
                    lastPosX = next_wp.transform.location.x
                    lastPosY = -next_wp.transform.location.y
                    last_wp = next_wp

                intersecion_geometry.lane_set.array.append(generic_lane_egress)
                    
                    
            mapem.map.intersections_is_present = True
            mapem.map.intersections.array.append(intersecion_geometry)
            
        return mapem
    
    @staticmethod
    def create_tesi_spatem_message(junctions):
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
        print("Starting traffic sensor")
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
        
