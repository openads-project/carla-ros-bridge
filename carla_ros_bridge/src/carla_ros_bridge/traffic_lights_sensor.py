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
            "carla_converter/mapem",
            qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.etsi_spatem_publisher = node.new_publisher(
            SPATEM,
            "carla_converter/spatem",
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
    def convert_lane_type(carla_lane_type : LaneType):
        lane_actions = {
            LaneType.NONE: 0,
            LaneType.Driving: LaneTypeAttributes.vehicle,
            LaneType.Stop: 0,
            LaneType.Shoulder: 0,
            LaneType.Biking: LaneTypeAttributes.bike_lane,
            LaneType.Sidewalk: LaneTypeAttributes.sidewalk,
            LaneType.Border: 0,
            LaneType.Restricted: 0,
            LaneType.Parking: LaneTypeAttributes.parking,
            LaneType.Bidirectional: 0,
            LaneType.Median: LaneTypeAttributes.median,
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
                            'waypoint_taffic_light_dict': {}
                        }
                        
                        waypoints = junction_object.get_waypoints(LaneType.Any)
                        
                        for waypoint in waypoints:
                            junctions[junction_id]['waypoints_tuple'].append(waypoint)
                        
                    junctions[junction_id]['traffic_lights'][traffic_light.uid] = traffic_light
                    junctions[junction_id]['waypoint_taffic_light_dict'][waypoints_traffic_light[0].id] = traffic_light
                        
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
        
            for waypointTuple in junctionContainer['waypoints_tuple']:
                
                # create GenericLane
                waypoint = waypointTuple[0]
                
                generic_lane = GenericLane()
                generic_lane.lane_id.value = waypoint.road_id   
                #generic_lane.maneuvers_is_present = waypoint.lane_change != LaneChange.NONE
                #generic_lane.maneuvers.value = waypoint.lane_change
                generic_lane._lane_attributes.lane_type.choice = TrafficLightsSensor.convert_lane_type(waypoint.lane_type)
                
                generic_lane.lane_attributes.directional_use.value.append(192)
                generic_lane.lane_attributes.directional_use.bits_unused = 6
                
                # set connection to traffic light if available
                if waypoint.id in junctionContainer['waypoint_taffic_light_dict']:
                    traffic_light = junctionContainer['waypoint_taffic_light_dict'][waypoint.id]
                    connection = Connection()
                    
                    connection.signal_group_is_present = True
                    connection.signal_group.value = traffic_light.uid
                    
                    generic_lane.connects_to_is_present = True
                    generic_lane.connects_to.array.append(connection)
                    # generic_lane.connections.append(connection) todo: where is the difference here?


                # lane consists of a nodelist of 2 nodes
                generic_lane.node_list = NodeListXY()
                generic_lane.node_list.choice = NodeListXY.CHOICE_NODES
                
                wp1, wp2 = waypointTuple
                
                posAbsX = wp1.transform.location.x
                posAbsY = wp1.transform.location.y
                posAbsZ = wp1.transform.location.z
                
                posDeltaX = wp2.transform.location.x - posAbsX
                posDeltaY = wp2.transform.location.y - posAbsY
                posDeltaZ = wp2.transform.location.z - posAbsZ
                
                node1 = NodeXY()
                node1.delta.node_xy1.x.value = posAbsX * 100
                node1.delta.node_xy1.y.value = posAbsY * 100
                node1.attributes.d_elevation_is_present = True
                node1.attributes.d_elevation.value = posAbsZ * 100
                
                node2 = NodeXY()
                node2.delta.node_xy1.x.value = posDeltaX * 100
                node2.delta.node_xy1.y.value = posDeltaY * 100
                node2.attributes.d_elevation_is_present = True
                node2.attributes.d_elevation.value = posDeltaZ * 100

                #generic_lane.node_list.nodes.array.append(node1)
                #generic_lane.node_list.nodes.array.append(node2)
                    
                intersecion_geometry.lane_set.array.append(generic_lane)
                
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
        
        
        #for actor in traffic_light_actors:
        #    if isinstance(actor, TrafficLight):
        #        traffic_light = actor  # Now it's cast to TrafficLight
        #        status = traffic_light.get_status()
        #        
        #        # Intersection State
        #        intersection_state = IntersectionState()
        #        intersection_state.revision.value = 0 # are we interested in getting a revision from the CARLA simulation, e.g. a version number of the scene?
        #        
        #        # Movement State
        #        movement_state = MovementState()
        #        movement_state.signal_group.value = traffic_light.uid
        #        
        #        # Movement event
        #        movement_event = MovementEvent()
        #        movement_event.event_state.value = TrafficLightsSensor.convert_traffic_light_state(status.state)
        #        
        #        # fill arrays
        #        movement_state.state_time_speed.array.append(movement_event)
        #        intersection_state.states.array.append(movement_state)
        #        spatem.spat.intersections.array.append(intersection_state)