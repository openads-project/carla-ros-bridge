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
import pyproj
import rclpy
from rclpy.time import Time
from collections import namedtuple
from ros_compatibility.qos import QoSProfile, DurabilityPolicy
import numpy as np

from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs

from carla_ros_bridge.pseudo_actor import PseudoActor
from carla_ros_bridge.traffic import TrafficLight

from carla_msgs.msg import CarlaTrafficLightStatusList, CarlaTrafficLightInfoList

from carla_msgs.msg import CarlaTrafficLightStatus
from carla import LaneType

from etsi_its_mapem_ts_msgs.msg import (
    MAPEM,
    Connection,
    GenericLane,
    IntersectionGeometry,
    LaneDirection,
    LaneTypeAttributes,
    NodeListXY,
    NodeXY,
)
from etsi_its_spatem_ts_msgs.msg import (
    SPATEM,
    IntersectionState,
    MovementEvent,
    MovementPhaseState,
    MovementState,
)

class TrafficLightsSensor(PseudoActor):
    """
    a sensor that reports the state of all traffic lights
    """

    """"
    Ingress Lane data container
    :param lane_id: unique identifier for this lane. The lane_id is unique for each lane inside a junction and is genetrated from 0 to n-1, where n is the number of lanes inside the junction
    :type lane_id: int
    :param lane_type: type of the lane as defined in the ETSI standard
    :type lane_type: int
    :param waypoint_junction: entry waypoint from ingress lane into the junction
    :type waypoint_junction: carla.Waypoint
    :param traffic_light: correponding traffic light object if available for ETSI lane
    :type traffic_light: carla.TrafficLight
    :param connected_egress_lane_ids: list of ids of all connected egress lanes
    :type connected_egress_lane_ids: list
    """
    Ingress_Lane = namedtuple(
        "Ingress_Lane",
        ["lane_id", "lane_type", "waypoint_junction", "traffic_light", "connected_egress_lane_ids"],
    )
    
    
    """"
    Egress Lane data container
    :param lane_id: unique identifier for this lane. The lane_id is unique for each lane inside a junction and is genetrated from 0 to n-1, where n is the number of lanes inside the junction
    :type lane_id: int
    :param lane_type: type of the lane as defined in the ETSI standard
    :type lane_type: int
    :param waypoint_junction: exit waypoint from junction into the egress lane
    :type waypoint_junction: carla.Waypoint
    """
    Egress_Lane = namedtuple(
        "Egress_Lane",
        ["lane_id", "lane_type", "waypoint_junction"],
    )

    def __init__(self, uid, name, parent, node, actor_list, tf_buffer):
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
        :param tf_buffer: shared transform buffer owned by the bridge node
        :type tf_buffer: tf2_ros.Buffer
        """

        super(TrafficLightsSensor, self).__init__(
            uid=uid, name=name, parent=parent, node=node
        )
        self.node = node
        self.actor_list = actor_list
        self.traffic_light_status = CarlaTrafficLightStatusList()
        self.traffic_light_actors = []
        self.carla_to_utm_rotation_matrix_initialized = False
        self._mapem_publish_warned = False
        self.etsi_mapem_publisher = None
        self.etsi_spatem_publisher = None
        self.tf_buffer = tf_buffer

        self.publish_etsi_messages = node.parameters["publish_etsi_messages"]
        self.waypoints_search_distance = node.parameters["waypoints_search_distance"]
        self.lane_waypoints_count = node.parameters["lane_waypoints_count"]

        self.traffic_lights_info_publisher = node.new_publisher(
            CarlaTrafficLightInfoList,
            self.get_topic_prefix() + "/info",
            qos_profile=QoSProfile(
                depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )
        self.traffic_lights_status_publisher = node.new_publisher(
            CarlaTrafficLightStatusList,
            self.get_topic_prefix() + "/status",
            qos_profile=QoSProfile(
                depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )

        if self.publish_etsi_messages:
            if self.tf_buffer is None:
                raise ValueError("TrafficLightsSensor requires a shared tf_buffer when ETSI messages are enabled")
            traffic_light_actors = self.get_traffic_light_actors()
            self.initialize_junctions(traffic_light_actors)

            self.etsi_mapem_publisher = node.new_publisher(
                MAPEM,
                "/carla/etsi/mapem",
                qos_profile=QoSProfile(
                    depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL
                ),
            )

            self.etsi_spatem_publisher = node.new_publisher(
                SPATEM,
                "/carla/etsi/spatem",
                qos_profile=QoSProfile(
                    depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL
                ),
            )

            # spatem publisher callback
            timer_period = node.parameters["mapem_timer_period"]
            self.timer_mapem = node.create_timer(
                timer_period, self.publish_etsi_mapem_message
            )

            # mapem publisher callback
            timer_period = node.parameters["spatem_timer_period"]
            self.timer_spatem = node.create_timer(
                timer_period, self.publish_etsi_spatem_message
            )

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        super(TrafficLightsSensor, self).destroy()
        self.actor_list = None
        self.node.destroy_publisher(self.traffic_lights_info_publisher)
        self.node.destroy_publisher(self.traffic_lights_status_publisher)

        if self.etsi_mapem_publisher is not None:
            self.node.destroy_publisher(self.etsi_mapem_publisher)

        if self.etsi_spatem_publisher is not None:
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
        Returns a junction with the given id. The id of the carla. Junction corresponds to the id of the junction in the OpenDRIVE file.
        :param junction_id: id of the junction
        :type junction_id: int
        :return Carla junction from the Carla world object
        :rtype carla.Junction
        """
        return self.junctions[junction_id]["junction_object"]

    def get_junction_ingress_lanes(self, junction_id):
        """
        Returns all ingress lanes that belong to a junction with a given id
        :param junction_id: id of the junction
        :type junction_id: int
        :return Ingress lanes of a junction
        :rtype array(namedTuple(Ingress_Lane))
        """
        return self.junctions[junction_id]["ingress_lanes"].values()
    
    def get_junction_egress_lanes(self, junction_id):
        """
        Returns all egress lanes that belong to a junction with a given id
        :param junction_id: id of the junction
        :type junction_id: int
        :return Egress lanes of a junction
        :rtype array(namedTuple(Egress_Lane))
        """
        return self.junctions[junction_id]["egress_lanes"].values()

    def get_junction_traffic_lights(self, junction_id):
        """
        Returns all traffic lights that belong to a junction with a given id
        :param junction_id: id of the junction
        :type junction_id: int
        :return Traffic lights of a junction
        :rtype array(carla.Junction)
        """
        return self.junctions[junction_id]["traffic_lights"].values()

    def get_junction_position(self, junction_id):
        """
        Returns the position of a junction with a given id
        :param junction_id: id of the junction
        :type junction_id: int
        :return the mean position of all edge waypoints of the junction
        :rtype numpy.array(3)
        """

        return self.junctions[junction_id]["position"]

    def set_junction(self, junction_id, value):
        """
        Stores a junction with the given id. The id of the carla. Junction corresponds to the id of the junction in the OpenDRIVE file.
        :param junction_id: id of the junction
        :type junction_id: int
        :param value: carla junction object
        :type value: carla.Junction
        """

        self.junctions[junction_id]["junction_object"] = value

    def set_junction_ingress_lanes(self, junction_id, value):
        """
        Stores all ingress lanes that belong to a junction with a given id
        :param junction_id: id of the junction to which the ingress lanes belong
        :type junction_id: int
        :param value: Ingress lanes of a junction
        :type value: array(namedTuple(Ingress_Lane))
        """
        
        self.junctions[junction_id]["ingress_lanes"] = value
        
    def set_junction_egress_lanes(self, junction_id, value):
        """
        Stores all Egress lanes that belong to a junction with a given id
        :param junction_id: id of the junction to which the Egress lanes belong
        :type junction_id: int
        :param value: Egress lanes of a junction
        :type value: array(namedTuple(Egress_Lane))
        """

        self.junctions[junction_id]["egress_lanes"] = value
        

    def set_junction_traffic_lights(self, junction_id, value):
        """
        Stores all traffic lights that belong to a junction with a given id
        :param junction_id: id of the junction to which the traffic lights belong
        :type junction_id: int
        :param value: traffic lights of a junction
        :type value: array(carla.Junction)
        """

        self.junctions[junction_id]["traffic_lights"] = value

    def set_junction_position(self, junction_id, value):
        """
        Returns the position of a junction with a given id
        :param junction_id: id of the junction
        :type junction_id: int
        :param value: the mean position of all edge waypoints of the junction
        :type value: numpy.array(3)
        """

        self.junctions[junction_id]["position"] = value

    @staticmethod
    def set_etsi_lat_lon_junction(etsi_junction, lat, lon, z):
        """
        Sets the lat lon WGS84 coordinates for a intersection inside an ETSI message and scales the values accordingly
        :param etsi_junction: id of the junction
        :type etsi_junction: IntersectionGeometry
        :param value: lat value of the intersection center as WGS84
        :type value: float
        :param value: lon value of the intersection center as WGS84
        :type value: float
        :param value: height of the intersection center
        :type value: float
        """

        etsi_junction.ref_point.lat.value = (int)(lat * 10**7)
        etsi_junction.ref_point.lon.value = (int)(lon * 10**7)
        etsi_junction.ref_point.elevation.value = (int)(z * 10**1)

    @staticmethod
    def convert_carla_location_to_ros_vector3(location):
        """
        Helper method which converts a position from carla coordinates into ROS2 coordinates
        :param location: location/position to transform
        :type location: carla.Location
        :param value: transformed position Vector3
        :type value: np.Array(3)
        """

        return np.array([location.x, -location.y, location.z])

    @staticmethod
    def carla_to_latlon(projection_string, carla_x, carla_y):
        """
        Convert CARLA coordinates to latitude/longitude using PyProj and a reference point.
        :param projection_string: Projection string which is used to convert Carla coordinates to lat/lon coordinates
        :type projection_string: string
        :param carla_x: CARLA x coordinate to convert
        :type carla_x: float
        :param carla_y: CARLA y coordinate to convert
        :type carla_y: float
        :return: latitude, longitude WGS84 coordinates
        :rtype: tuple(float, float)
        """

        proj_xodr = pyproj.Proj(projparams=projection_string)
        lon, lat = proj_xodr(carla_x, carla_y, inverse=True)

        return lat, lon

    @staticmethod
    def convert_lane_type(carla_lane_type: LaneType):
        """
        Convert a CARLA LaneType into an ETSI LaneType
        :param carla_lane_type: LaneType as Carla Type
        :type carla_lane_type: carla.LaneType
        :return: LaneType as ETSI type
        :rtype: int
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
            LaneType.Any: 0,
        }

        return lane_actions[carla_lane_type]

    @staticmethod
    def encode_single_bit_as_byte(bit_index):
        """
        Encode a named ETSI bit index as the corresponding byte value.
        ETSI bit strings are encoded most-significant bit first.
        """

        return 1 << (7 - bit_index)

    @staticmethod
    def convert_traffic_light_state(state: CarlaTrafficLightStatus):
        """
        Convert the type CarlaTrafficLightStatus into the corresponding ETSI type
        :param state: The current Carla state of the traffic light
        :type state: carla.TrafficLightState
        :return: The current traffic light as ETSI type
        :rtype: MovementPhaseState
        """

        state_dictionary = {
            CarlaTrafficLightStatus.RED: MovementPhaseState.STOP_THEN_PROCEED,
            CarlaTrafficLightStatus.YELLOW: MovementPhaseState.PRE_MOVEMENT,
            CarlaTrafficLightStatus.GREEN: MovementPhaseState.PERMISSIVE_MOVEMENT_ALLOWED,
            CarlaTrafficLightStatus.OFF: MovementPhaseState.DARK,
            CarlaTrafficLightStatus.UNKNOWN: MovementPhaseState.DARK,
        }

        return state_dictionary[state]

    @staticmethod
    def add_lane_node(lane, position):
        """
        Adds a new node to the given ETSI Mapem lane.
        :param lane: GenericLane ETSI Mapem object to which a new Node will be added
        :type lane: GenericLane
        :param position: position of the new node. Absolute in map frame if it is the first node in the lane, otherwise relative to it's predecessor node
        :type position: array(carla.TrafficLight)
        """

        node = NodeXY()
        node.delta.node_xy1.x.value = (int)(position[0] * 100)
        node.delta.node_xy1.y.value = (int)(position[1] * 100)
        lane.node_list.nodes.array.append(node)

    def check_is_initialized(self):
        """
        Check if other modules (Carla world) are initialized 
        """
        return self.node.world_info.transform_utm_to_carla != None

    def rotate_point_from_map_to_utm_frame(self, point_map):
        """
        # transforms a point from the map frame into the UTM world frame
        # the point is already converted to the right handed carla map ROS frame from the left handed CARLA frame
        
        :param point_map: point in CARLA map coordinates (converted into right hand system)
        :type point_map: numpy.array(3)
        :return tranformed point in UTM frame 
        :rtype numpy.array(3)
        """
        point_stamped_map = PointStamped()
        point_stamped_map.point.x = point_map[0]
        point_stamped_map.point.y = point_map[1]
        point_stamped_map.point.z = point_map[2]
        
        if not self.carla_to_utm_rotation_matrix_initialized:            
            world_info = self.node.world_info
            
            self.inverse_transform = self.tf_buffer.lookup_transform(
                world_info.world_frame,  # target frame
                world_info.map_frame,  # source frame
                Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            self.carla_to_utm_rotation_matrix_initialized = True
            
        point_utm = tf2_geometry_msgs.do_transform_point(point_stamped_map, self.inverse_transform)
        
        return np.array([point_utm.point.x, point_utm.point.y, point_utm.point.z])
        
    def initialize_junctions(self, traffic_lights):
        """
        Initializes the junctions of the Carla world. For each junction, all traffic lights and their corresponding lanes are stored.
        :param traffic_lights: all traffic light actors from the carla world
        :type traffic_lights: array(carla.TrafficLight)
        """
        self.junctions = {}

        affected_lane_index = self.build_affected_lane_index(traffic_lights)

        # iterate through all junction candidates inside and cache them with potential traffic lights
        for junction_object in self.get_all_junctions_from_world().values():
            junction_id = junction_object.id

            if junction_id not in self.junctions:

                # get all waypoint tuples for a given junction
                # a tuple represents a driving line from the entrance to a junction ingress -first tuple element)
                # to an exit (egress - second tuple element)
                # an ingress lane can have an attached traffic light
                waypoint_tuples = junction_object.get_waypoints(LaneType.Driving)
                junction_traffic_lights = {}
                junction_position = TrafficLightsSensor.calculate_junction_mean(
                    waypoint_tuples
                )

                ingress_lanes = {}
                egress_lanes = {}
                
                # connect traffic light with corresponding ingress lane waypoint if available
                lane_id_counter = 0

                for waypoint_tuple in waypoint_tuples:
                    junction_entry_waypoint, junction_exit_waypoint = waypoint_tuple

                    traffic_light = self.get_ingress_traffic_light(
                        junction_entry_waypoint, affected_lane_index
                    )

                    if traffic_light is not None:
                        junction_traffic_lights[traffic_light.id] = traffic_light

                    # create and update ingress lanes
                    if junction_entry_waypoint.id not in ingress_lanes:
                        # connect the ingress lane with the traffic light and all junction entry waypoints on the observed lane
                        #ingress_lanes[ingress_lane_waypoint.road_id] = (ingress_lane_waypoint, traffic_light, [])
                        ingress_lanes[junction_entry_waypoint.id] = self.Ingress_Lane(
                            lane_id = lane_id_counter,
                            lane_type=TrafficLightsSensor.convert_lane_type(
                                junction_entry_waypoint.lane_type),
                            waypoint_junction=junction_entry_waypoint,
                            traffic_light=traffic_light,
                            connected_egress_lane_ids=[])
                        
                        lane_id_counter += 1

                    # create and update egress lanes                    
                    if junction_exit_waypoint.id not in egress_lanes:
                        # connect the egress lane with all junction exit waypoints on the observed lane
                        #egress_lanes[egress_lane_waypoint.road_id] = (egress_lane_waypoint, [])
                        egress_lanes[junction_exit_waypoint.id] = self.Egress_Lane(
                            lane_id = lane_id_counter,
                            lane_type=TrafficLightsSensor.convert_lane_type(
                                junction_exit_waypoint.lane_type),
                            waypoint_junction=junction_exit_waypoint)

                        lane_id_counter += 1

                # iterate again to connect the ingress lanes with the corresponding egress lanes
                for waypoint_tuple in waypoint_tuples:
                    junction_entry_waypoint, junction_exit_waypoint = waypoint_tuple
                    
                    egress_lane_id = egress_lanes[junction_exit_waypoint.id].lane_id
                    ingress_lanes[junction_entry_waypoint.id].connected_egress_lane_ids.append(egress_lane_id)
                
                # fill junction data structure
                if len(junction_traffic_lights) > 0:
                    self.junctions[junction_id] = {}
                    self.set_junction(junction_id, junction_object)
                    self.set_junction_traffic_lights(
                        junction_id, junction_traffic_lights
                    )
                    self.set_junction_position(junction_id, junction_position)
                    self.set_junction_ingress_lanes(junction_id, ingress_lanes)
                    self.set_junction_egress_lanes(junction_id, egress_lanes)

    def build_affected_lane_index(self, traffic_lights):
        """
        Maps the (road_id, lane_id) of every lane affected by a traffic light to that traffic light actor, using each light's OpenDRIVE affected lanes.
        :param traffic_lights: all traffic light actors from the carla world
        :type traffic_lights: array(carla.TrafficLight)
        :return mapping of (road_id, lane_id) to the affecting traffic light actor
        :rtype dict(tuple(int, int), carla.TrafficLight)
        """
        affected_lane_index = {}

        for traffic_light in traffic_lights:
            actor = traffic_light.carla_actor
            affected_waypoints = actor.get_affected_lane_waypoints()

            if len(affected_waypoints) == 0:
                self.node.logwarn(
                    "Traffic light {} affects no lanes. Is it defined as an "
                    "OpenDRIVE signal with lane validities?".format(actor.id)
                )

            for waypoint in affected_waypoints:
                affected_lane_index[(waypoint.road_id, waypoint.lane_id)] = actor

        return affected_lane_index

    def get_ingress_traffic_light(self, junction_entry_waypoint, affected_lane_index):
        """
        Returns the traffic light controlling the given junction ingress lane, or None. The approach lane feeding the junction is the entry waypoint's predecessor; the light is looked up by that lane's (road_id, lane_id).
        :param junction_entry_waypoint: entry waypoint of a junction connecting lane
        :type junction_entry_waypoint: carla.Waypoint
        :param affected_lane_index: mapping of (road_id, lane_id) to traffic light actor
        :type affected_lane_index: dict(tuple(int, int), carla.TrafficLight)
        :return the controlling traffic light actor or None
        :rtype carla.TrafficLight or None
        """
        for approach_waypoint in junction_entry_waypoint.previous(
            self.waypoints_search_distance
        ):
            traffic_light = affected_lane_index.get(
                (approach_waypoint.road_id, approach_waypoint.lane_id)
            )

            if traffic_light is not None:
                return traffic_light

        return None
    
    def get_all_junctions_from_world(self):
        """
        Returns a dictionary of id-junction pairs of all junctions in the Carla world
        :return dictionary of ids and the corresponding junctions
        :rtype dictionary(int, carla.Junction)
        """
        
        map = self.node.carla_world.get_map()
        all_waypoints = map.generate_waypoints(self.waypoints_search_distance)
        junctions = {}

        for waypoint in all_waypoints:
            if waypoint.is_junction:
                junction_id = waypoint.junction_id
                junction_object = waypoint.get_junction()

                if junction_id not in junctions:
                    junctions[junction_id] = junction_object
                    
        return junctions

    def create_junction_lane(self, lane_id, lane_type, is_ingress, waypoint, junction_position):
        """
        Creates an Ingress or Egress lane.
        The lane is part of an ETSI Mapem message.
        :param is_ingress: The lane is an Ingress lane (true, enters the intersection) or an Egress lane (false, exits the intersection)
        :type is_ingress: bool
        :param waypoint: the waypoint corresponding to the lane at the start/end of the intersection
        :type waypoint: carla.Waypoint
        :param junction_position: position of the junction
        :type junction_position: numpy.array(3)
        :return ETSI Mapem lane (Ingress or Egress)
        :rtype GenericLane
        """
        
        # create ingress line for
        generic_lane = GenericLane()
        generic_lane.lane_id.value = lane_id
        generic_lane._lane_attributes.lane_type.choice = lane_type     

        lane_direction = generic_lane.lane_attributes.directional_use
        lane_direction_bit_index = (
            LaneDirection.BIT_INDEX_INGRESS_PATH
            if is_ingress else LaneDirection.BIT_INDEX_EGRESS_PATH
        )
        lane_direction.value.append(
            self.encode_single_bit_as_byte(lane_direction_bit_index)
        )
        lane_direction.bits_unused = 8 - LaneDirection.SIZE_BITS

        # lane consists of a nodelist of two nodes
        generic_lane.node_list = NodeListXY()
        generic_lane.node_list.choice = NodeListXY.CHOICE_NODES
            
        pos_abs = TrafficLightsSensor.convert_carla_location_to_ros_vector3(
            waypoint.transform.location
        )
        
        junction_position_utm = self.rotate_point_from_map_to_utm_frame(junction_position)
        pos_abs_utm = self.rotate_point_from_map_to_utm_frame(pos_abs)
        
        pos_rel_junction_utm = pos_abs_utm - junction_position_utm
        TrafficLightsSensor.add_lane_node(generic_lane, pos_rel_junction_utm)

        last_wp = waypoint
        last_pos = pos_abs_utm

        # create an egress/ingress lane with a given length
        for i in range(self.lane_waypoints_count):
            if is_ingress:
                next_wps = last_wp.previous(self.waypoints_search_distance)
            else:
                next_wps = last_wp.next(self.waypoints_search_distance)

            if len(next_wps) == 0:
                break

            next_wp = next_wps[0]
            next_wp_position = (
                TrafficLightsSensor.convert_carla_location_to_ros_vector3(
                    next_wp.transform.location
                )
            )
            
            next_wp_position_utm = self.rotate_point_from_map_to_utm_frame(next_wp_position)

            pos_rel = next_wp_position_utm - last_pos
            TrafficLightsSensor.add_lane_node(generic_lane, pos_rel)
            last_pos = next_wp_position_utm
            last_wp = next_wp

        return generic_lane

    @staticmethod
    def calculate_junction_mean(junction_waypoint_tuples):
        """
        Calculates the position of a junction by using the mean position of all edge waypoints.
        :param junction_waypoint_tuples: all driving lane waypoints from the edge of the junction (ingoing and outgoing)
        :return mean position of all positions from the given lane intersection tuples
        :rtype numpy.array(3)
        """
        position = np.array([0.0, 0.0, 0.0])
        waypoint_count = 0

        # set the lat/lon coordinates of junction as mean of corresponding traffic light positions
        for waypoint_tuple in junction_waypoint_tuples:
            entry_waypoint, exit_waypoint = waypoint_tuple
            
            position = (
                position
                + TrafficLightsSensor.convert_carla_location_to_ros_vector3(
                    entry_waypoint.transform.location
                )
            )
            position = (
                position
                + TrafficLightsSensor.convert_carla_location_to_ros_vector3(
                    exit_waypoint.transform.location
                )
            )

            waypoint_count += 2

        if waypoint_count > 0:
            position = position / waypoint_count

        return position

    def publish_etsi_mapem_message(self):
        """
        Creates and publishes an ETSI Mapem message
        """

        if not self.check_is_initialized():
            return

        try:
            # create MAPEM data
            mapem = MAPEM()
            mapem.map.msg_issue_revision.value = 0

            for junction_id in self.junctions:
                junction = self.get_junction(junction_id)
                junction_position = self.get_junction_position(junction_id)
                junction_ingress_lanes = self.get_junction_ingress_lanes(junction_id)
                junction_egress_lanes = self.get_junction_egress_lanes(junction_id)

                # create intersection geometry
                intersection_geometry = IntersectionGeometry()
                intersection_geometry.id.id.value = junction.id
                intersection_geometry.ref_point.elevation_is_present = True

                # set the lat/lon coordinates of junction as mean of corresponding traffic light positions
                projection_string = self.node.world_info.projection_string
                lat, lon = TrafficLightsSensor.carla_to_latlon(
                    projection_string, junction_position[0], junction_position[1]
                )
                TrafficLightsSensor.set_etsi_lat_lon_junction(
                    intersection_geometry, lat, lon, junction_position[2]
                )

                # create junction ingress lanes
                for ingress_lane in junction_ingress_lanes:

                    # create ingress lane which lead through the junction into an egress lane
                    generic_lane_ingress = self.create_junction_lane(
                        ingress_lane.lane_id, ingress_lane.lane_type, True, ingress_lane.waypoint_junction, junction_position
                    )

                    intersection_geometry.lane_set.array.append(generic_lane_ingress)

                    # connect Ingress lanes to Egess lanes and the traffic light signal, if available
                    generic_lane_ingress.connects_to_is_present = True

                    for egress_lane_id in ingress_lane.connected_egress_lane_ids:
                        connection = Connection()

                        if ingress_lane.traffic_light != None:
                            connection.signal_group_is_present = True
                            connection.signal_group.value = ingress_lane.traffic_light.id

                        # add the lane of the connection
                        connection.connecting_lane.lane.value = egress_lane_id
                        generic_lane_ingress.connects_to.array.append(connection)

                # create junction egress lanes
                for egress_lane in junction_egress_lanes:
                    # create egress lane
                    generic_lane_egress = self.create_junction_lane(
                        egress_lane.lane_id, egress_lane.lane_type, False, egress_lane.waypoint_junction, junction_position
                    )

                    intersection_geometry.lane_set.array.append(generic_lane_egress)

                mapem.map.intersections_is_present = True
                mapem.map.intersections.array.append(intersection_geometry)

            self.etsi_mapem_publisher.publish(mapem)
            self._mapem_publish_warned = False
        except Exception as e:
            # keep node alive when TF data is temporarily unavailable
            self.carla_to_utm_rotation_matrix_initialized = False
            if not self._mapem_publish_warned:
                self.node.logwarn("Skipping ETSI MAPEM publish this cycle: {}".format(e))
                self._mapem_publish_warned = True

    def publish_etsi_spatem_message(self):
        """
        Creates and publishes an ETSI SPATEM message
        """

        if not self.check_is_initialized():
            return
        
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
                movement_event.event_state.value = (
                    TrafficLightsSensor.convert_traffic_light_state(traffic_light.state)
                )

                # fill arrays
                movement_state.state_time_speed.array.append(movement_event)
                intersection_state.states.array.append(movement_state)

            spatem.spat.intersections.array.append(intersection_state)

        self.etsi_spatem_publisher.publish(spatem)

    def update(self, frame, timestamp):
        """
        Get the state of all known traffic lights
        """        
        if not self.check_is_initialized():
            return
        
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
