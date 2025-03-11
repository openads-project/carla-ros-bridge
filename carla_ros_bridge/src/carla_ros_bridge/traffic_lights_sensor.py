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
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_ros_bridge.pseudo_actor import PseudoActor
from carla_ros_bridge.traffic import TrafficLight

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
            "/etsi_its_conversion/mapem_ts/out",
            qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.etsi_spatem_publisher = node.new_publisher(
            SPATEM,
            "/etsi_its_conversion/spatem_ts/out",
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
            

        
        ## create Etsi data structure
        junctions = {}
        
        for actor in traffic_light_actors:
            if isinstance(actor, TrafficLight):
                traffic_light = actor  # Now it's cast to TrafficLight
                info = traffic_light.get_info()
                
                print("Traffic with id: ", traffic_light.uid,
                    ", x: ", info.transform.position.x,
                    ", y: ", info.transform.position.y,
                    ", z: ", info.transform.position.z
                )
                
                waypoints_traffic_light = traffic_light.carla_actor.get_affected_lane_waypoints()
                
                #for waypoint in waypoints:
                waypoint = waypoints_traffic_light[0]
                
                if waypoint.is_junction:
                    junction_id = waypoint.junction_id
                    
                    
                    print( "Junction id:", waypoint.junction_id, ", lane id:", waypoint.lane_id, 
                            " lane type: ", waypoint.lane_type, ", lane change: "
                            , waypoint.lane_change, ", x: ", waypoint.transform.location.x
                            , waypoint.lane_change, ", y: ", waypoint.transform.location.y
                            , waypoint.lane_change, ", z: ", waypoint.transform.location.z)
                    
                    # Create new junction entry if it doesn't exist
                    if junction_id not in junctions:
                        # Try to get the actual junction object
                        junction_object = waypoint.get_junction()
                        
                        junctions[junction_id] = {
                            'junction_object': junction_object,
                            'traffic_lights': {},
                            'waypoints_tuple': [],
                            'waypoint_traffic_light_dict': {}
                        }
                        
                        waypoints = junction_object.get_waypoints(LaneType.Driving)
                        
                        for waypoint in waypoints:
                            junctions[junction_id]['waypoints_tuple'].append(waypoint)
                        
                    junctions[junction_id]['traffic_lights'][traffic_light.uid] = traffic_light
                    junctions[junction_id]['waypoint_traffic_light_dict'][waypoints_traffic_light[0].id] = traffic_light
                        
                print ("__")
                
                            

        print("PRINTN RESULTS")        
                
        for junctionKey in junctions:
            junctionObject = junctions[junctionKey]
            junction = junctionObject['junction_object']
            
            print("Junction ID: ", junction.id)
            
            print("  Traffic Lights:")
            for tl in junctionObject['traffic_lights'].values():
                info = tl.get_info()
        
                print("Id: ", tl.uid,
                    ", x: ", info.transform.position.x,
                    ", y: ", info.transform.position.y,
                    ", z: ", info.transform.position.z
                )
        
            print("Waypoints:")
            for waypointTuple in junctionObject['waypoints_tuple']:
                for waypoint in waypointTuple:
                    print( "Id: ", waypoint.id, "lane id:", waypoint.lane_id, "Road id: ", waypoint.road_id, "section id: ", waypoint.section_id,
                                " lane type: ", waypoint.lane_type, ", lane change: ", waypoint.lane_change, 
                                ", x: ", waypoint.transform.location.x,
                                ", y: ", waypoint.transform.location.y,
                                ", z: ", waypoint.transform.location.z)
            
            
            print()
            print("_______")
            print()
            
        print()
                
        # create mapem mesage
        mapem = MAPEM()
        mapem.map.msg_issue_revision.value = 0
        
        for junctionKey in junctions:
            junctionContainer = junctions[junctionKey]
            junction = junctionContainer['junction_object']
            
            # Create intersection geometry
            intersecion_geometry = IntersectionGeometry()
            intersecion_geometry.id.id.value = junction.id
            intersecion_geometry.ref_point.lat.value = 0 # todo
            intersecion_geometry.ref_point.lon.value = 0 # todo
            intersecion_geometry.ref_point.elevation_is_present = True
            intersecion_geometry.ref_point.elevation.value = 0
        
            junctionPosX = 0
            junctionPosY = 0
            junctionCount = 0
            
            for traffic_light in junctionContainer['traffic_lights'].values():
                    waypoints = traffic_light.carla_actor.get_affected_lane_waypoints()
                    wp = waypoints[0]
                    
                    junctionPosX += wp.transform.location.x
                    junctionPosY += -wp.transform.location.y
                    junctionCount += 1
                    
            junctionPosX = junctionPosX / junctionCount
            junctionPosY = junctionPosY / junctionCount 
            
            
            #if junctionCount > 0:
            #    intersecion_geometry.ref_point.lat.value = (int)(junctionPosX * 100)
            #    intersecion_geometry.ref_point.lon.value = (int)(junctionPosY * 100)
        
            if True:
                for traffic_light in junctionContainer['traffic_lights'].values():
                    info = traffic_light.get_info()
                    waypoints = traffic_light.carla_actor.get_affected_lane_waypoints()
                    
                    generic_lane = GenericLane()
                    generic_lane.lane_id.value = waypoint.road_id   
                    
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
                    
                    wp1 = waypoints[0]
                    wp2 = waypoints[1]
                    
                    posAbsX = wp1.transform.location.x
                    posAbsY = wp1.transform.location.y
                    posAbsZ = wp1.transform.location.z
                    
                    posAbsX = info.transform.position.x
                    posAbsY = info.transform.position.y
                    posAbsZ = info.transform.position.z
                    
                    posDeltaX = wp2.transform.location.x - posAbsX
                    posDeltaY = wp2.transform.location.y - posAbsY
                    posDeltaZ = wp2.transform.location.z - posAbsZ
                    
                    posDeltaX = 0#info.trigger_volume.center.x
                    posDeltaY = 0#info.trigger_volume.center.y
                    posDeltaZ = 0#info.trigger_volume.center.z
                    
                    node1 = NodeXY()
                    
                    node1.delta.node_xy1.x.value = (int)(posAbsX * 100)
                    node1.delta.node_xy1.y.value = (int)(posAbsY * 100)
                    node1.attributes.d_elevation_is_present = True
                    node1.attributes.d_elevation.value = (int)(posAbsZ * 100)
                    
                    node2 = NodeXY()
                    node2.delta.node_xy1.x.value = (int)(posDeltaX * 100)
                    node2.delta.node_xy1.y.value = (int)(posDeltaY * 100)
                    node2.attributes.d_elevation_is_present = True
                    node2.attributes.d_elevation.value = (int)(posDeltaZ * 100)

                    generic_lane.node_list.nodes.array.append(node1)
                    generic_lane.node_list.nodes.array.append(node2)
                        
                    intersecion_geometry.lane_set.array.append(generic_lane)
                    
                    
                    # add affected wps
                    if True:
                        generic_lane2 = GenericLane()
                        generic_lane2.lane_id.value = waypoint.road_id   
                        
                        generic_lane2.lane_attributes.directional_use.value.append(192)
                        generic_lane2.lane_attributes.directional_use.bits_unused = 6
                    
                        
                        # lane consists of a nodelist of 2 nodes
                        generic_lane2.node_list = NodeListXY()
                        generic_lane2.node_list.choice = NodeListXY.CHOICE_NODES
                        
                        lastX = 0
                        lastY = 0
                        
                        for wp in waypoints:
                            posDeltaX = wp2.transform.location.x - lastX
                            posDeltaY = -wp2.transform.location.y - lastY
                            
                            node1 = NodeXY()
                        
                            node1.delta.node_xy1.x.value = (int)(posDeltaX * 100)
                            node1.delta.node_xy1.y.value = (int)(posDeltaY * 100)
                            
                            lastX = wp.transform.location.x
                            lastY = -wp.transform.location.y
                            
                            generic_lane2.node_list.nodes.array.append(node1)

                        intersecion_geometry.lane_set.array.append(generic_lane2)    
                        
            if True:
                for waypointTuple in junctionContainer['waypoints_tuple']:
                    
                    # create GenericLane
                    waypoint = waypointTuple[0]
                    wp1, wp2 = waypointTuple
                    
                    generic_lane = GenericLane()
                    generic_lane.lane_id.value = waypoint.road_id   
                    #generic_lane.maneuvers_is_present = waypoint.lane_change != LaneChange.NONE
                    #generic_lane.maneuvers.value = waypoint.lane_change
                    generic_lane._lane_attributes.lane_type.choice = TrafficLightsSensor.convert_lane_type(waypoint.lane_type)

                    generic_lane.lane_attributes.directional_use.value.append(64)
                    generic_lane.lane_attributes.directional_use.bits_unused = 6
                    
                    # set connection to traffic light if available
                    is_traffic_light = False
                    traffic_light = None
                    
                    for traffic_light in junctionContainer['traffic_lights'].values():
                        waypoints = traffic_light.carla_actor.get_affected_lane_waypoints() 
                    
                        for wp in waypoints:
                            if wp.id == wp1.id or wp.id == wp2.id:
                                is_traffic_light = True
                                break
                    
                    #if waypoint.id in junctionContainer['waypoint_traffic_light_dict']:
                    if is_traffic_light and False:
                        #traffic_light = junctionContainer['waypoint_traffic_light_dict'][waypoint.id]
                        connection = Connection()
                        
                        connection.signal_group_is_present = True
                        connection.signal_group.value = traffic_light.uid
                        
                        generic_lane.connects_to_is_present = True
                        generic_lane.connects_to.array.append(connection)
                        
                        # generic_lane.connections.append(connection) todo: where is the difference here?


                    # lane consists of a nodelist of 2 nodes
                    generic_lane.node_list = NodeListXY()
                    generic_lane.node_list.choice = NodeListXY.CHOICE_NODES
                    
                    posAbsX = wp1.transform.location.x
                    posAbsY = -wp1.transform.location.y
                    posAbsZ = wp1.transform.location.z
                    
                    posDeltaX = wp2.transform.location.x - posAbsX
                    posDeltaY = -wp2.transform.location.y - posAbsY
                    posDeltaZ = wp2.transform.location.z - posAbsZ
                    
                    node1 = NodeXY()
                    
                    node1.delta.node_xy1.x.value = (int)(posAbsX * 100)
                    node1.delta.node_xy1.y.value = (int)(posAbsY * 100)
                    node1.attributes.d_elevation_is_present = True
                    node1.attributes.d_elevation.value = (int)(posAbsZ * 100)
                    
                    node2 = NodeXY()
                    node2.delta.node_xy1.x.value = (int)(posDeltaX * 100)
                    node2.delta.node_xy1.y.value = (int)(posDeltaY * 100)
                    node2.attributes.d_elevation_is_present = True
                    node2.attributes.d_elevation.value = (int)(posDeltaZ * 100)

                    generic_lane.node_list.nodes.array.append(node1)
                    generic_lane.node_list.nodes.array.append(node2)
                        
                    #intersecion_geometry.lane_set.array.append(generic_lane)
                    
                    
                    # create ingress line for
                    generic_lane_ingress = GenericLane()
                    generic_lane_ingress.lane_id.value = waypoint.road_id   
                    generic_lane_ingress._lane_attributes.lane_type.choice = TrafficLightsSensor.convert_lane_type(waypoint.lane_type)

                    generic_lane_ingress.lane_attributes.directional_use.value.append(128)
                    generic_lane_ingress.lane_attributes.directional_use.bits_unused = 6
                    
                    # lane consists of a nodelist of 2 nodes
                    generic_lane_ingress.node_list = NodeListXY()
                    generic_lane_ingress.node_list.choice = NodeListXY.CHOICE_NODES
                    
                    direction_x, direction_y = TrafficLightsSensor.get_lane_direction_vector(wp1)
                    posRelX = direction_x
                    posRelY = direction_y
                    
                    posAbsX = wp1.transform.location.x
                    posAbsY = -wp1.transform.location.y
                    posAbsZ = wp1.transform.location.z
                    
                    node1 = NodeXY()
                    
                    node1.delta.node_xy1.x.value = (int)(posAbsX * 100)
                    node1.delta.node_xy1.y.value = (int)(posAbsY * 100)

                    node2 = NodeXY()
                    node2.delta.node_xy1.x.value = (int)(direction_x * 500)
                    node2.delta.node_xy1.y.value = (int)(direction_y * 500)

                    generic_lane_ingress.node_list.nodes.array.append(node1)
                    #generic_lane_ingress.node_list.nodes.array.append(node2)
                        
                    last_wp = wp1
                    lastPosX = posAbsX
                    lastPosY = posAbsY
                    
                    for i in range(10):
                        next_wps = last_wp.previous(1.0)
                        
                        if len(next_wps) == 0:
                            break
                        
                        next_wp = next_wps[0]
                        posRelX = next_wp.transform.location.x - lastPosX
                        posRelY = -next_wp.transform.location.y - lastPosY
                        
                        node = NodeXY()
                        node.delta.node_xy1.x.value = (int)(posRelX * 100)
                        node.delta.node_xy1.y.value = (int)(posRelY * 100)
                        generic_lane_ingress.node_list.nodes.array.append(node)
                        
                        lastPosX = next_wp.transform.location.x
                        lastPosY = -next_wp.transform.location.y
                        last_wp = next_wp
                        
                        
                    intersecion_geometry.lane_set.array.append(generic_lane_ingress)
                    
                    
                    # create egress line for
                    generic_lane_ingress = GenericLane()
                    generic_lane_ingress.lane_id.value = waypoint.road_id   
                    generic_lane_ingress._lane_attributes.lane_type.choice = TrafficLightsSensor.convert_lane_type(waypoint.lane_type)

                    generic_lane_ingress.lane_attributes.directional_use.value.append(64)
                    generic_lane_ingress.lane_attributes.directional_use.bits_unused = 6
                    
                    # lane consists of a nodelist of 2 nodes
                    generic_lane_ingress.node_list = NodeListXY()
                    generic_lane_ingress.node_list.choice = NodeListXY.CHOICE_NODES
                    
                    direction_x, direction_y = TrafficLightsSensor.get_lane_direction_vector(wp2)
                    posRelX = direction_x
                    posRelY = direction_y
                    
                    posAbsX = wp2.transform.location.x
                    posAbsY = -wp2.transform.location.y
                    posAbsZ = wp2.transform.location.z
                    
                    node1 = NodeXY()
                    
                    node1.delta.node_xy1.x.value = (int)(posAbsX * 100)
                    node1.delta.node_xy1.y.value = (int)(posAbsY * 100)

                    node2 = NodeXY()
                    node2.delta.node_xy1.x.value = (int)(direction_x * 500)
                    node2.delta.node_xy1.y.value = (int)(direction_y * 500)

                    generic_lane_ingress.node_list.nodes.array.append(node1)
                    #generic_lane_ingress.node_list.nodes.array.append(node2)

                    last_wp = wp2
                    lastPosX = posAbsX
                    lastPosY = posAbsY
                    
                    for i in range(10):
                        next_wps = last_wp.next(1.0)
                        if len(next_wps) == 0:
                            break
                        
                        next_wp = next_wps[0]
                        posRelX = next_wp.transform.location.x - lastPosX
                        posRelY = -next_wp.transform.location.y - lastPosY
                        
                        node = NodeXY()
                        node.delta.node_xy1.x.value = (int)(posRelX * 100)
                        node.delta.node_xy1.y.value = (int)(posRelY * 100)
                        generic_lane_ingress.node_list.nodes.array.append(node)
                        
                        lastPosX = next_wp.transform.location.x
                        lastPosY = -next_wp.transform.location.y
                        last_wp = next_wp

                    intersecion_geometry.lane_set.array.append(generic_lane_ingress)
                    
                    
            mapem.map.intersections_is_present = True
            mapem.map.intersections.array.append(intersecion_geometry)



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
        
        #publish messages
        print("Try to publish mapem")
        self.etsi_mapem_publisher.publish(mapem)
        
        print ("Try to publish spatem")
        self.etsi_spatem_publisher.publish(spatem)
        