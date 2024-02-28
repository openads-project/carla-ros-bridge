#!/usr/bin/env python
#
# Copyright (c) 2024 Institute for Automotive Engineering (ika) RWTH Aachen University
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle an IdealObjectSensor
"""

from carla_ros_bridge.vehicle import Vehicle
from carla_ros_bridge.walker import Walker

from derived_object_msgs.msg import ObjectArray

from carla_ros_bridge.object_sensor import ObjectSensor

class IdealObjectSensor(ObjectSensor):

    """
    IdealObjectSensor
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

        # Extract relevant attributes and convert to float
        try:
            self.range = float(attributes["range"])
        except:
            self.range = 15.0
            self.node.logerr(
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

    def check_visibility(self, ego_vehicle, target): 

        # Get the location of the source parent vehicle of Ideal Object Sensor and the target
        ego_vehicle_location = ego_vehicle.carla_actor.get_location()
        target_location = target.carla_actor.get_location()
        
        # Calculate the Euclidean distance between the ego and the target 
        distance = ego_vehicle_location.distance(target_location)

        # Check if the target is inside the range of the sensor 
        if distance <= self.attributes["range"]:
            return True
        
        return False 
        
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
            - Get the vehicle that the IdealObjectSensor is appended
            - This can be either ego-vehicle or hero-vehicle based on the sensors.json definitions
        """
        ego_vehicle = self.actor_list[self.parent.uid]  

        for actor_id in self.actor_list.keys():
            # currently only Vehicles and Walkers are added to the object array
            if self.parent is None or self.parent.uid != actor_id:
                actor = self.actor_list[actor_id]
                if isinstance(actor, Vehicle) or isinstance(actor, Walker):
                    if self.check_visibility(ego_vehicle, actor):
                        ros_objects.objects.append(actor.get_object_info())

        self.object_publisher.publish(ros_objects)