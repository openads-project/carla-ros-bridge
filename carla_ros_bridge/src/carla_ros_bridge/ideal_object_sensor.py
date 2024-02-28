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
        for attribute in attributes:
            if attribute.key == "range":
                self.range = float(attribute.value)
                break

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

    def check_visibility(self, sensor_location, target_location): 

        # Calculate the Euclidean distance between the sensor and the target 
        distance = sensor_location.distance(target_location)

        # Check if the target is inside the range of the sensor 
        if distance <= self.range:
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
        ego_vehicle_location = ego_vehicle.carla_actor.get_location()

        # Iterate over all dynamic actors
        for actor_id in self.actor_list.keys():
            
            # Currently only vehicles and walkers are added to the object array
            if self.parent is None or self.parent.uid != actor_id:
                actor = self.actor_list[actor_id]
                if isinstance(actor, Vehicle) or isinstance(actor, Walker):
                    
                    # Get the location of the target
                    target_location = actor.carla_actor.get_location()

                    # Check visibility of the target
                    if self.check_visibility(ego_vehicle_location, target_location):
                        ros_objects.objects.append(actor.get_object_info())

        # Iterate over all static vehicles
        if(self.node.parameters['publish_static_vehicles']):
            for object_key, object_value in self.OBJECT_LABELS.items():
                static_vehicles = self.world.get_environment_objects(object_key)

                for vehicle in static_vehicles:
                    # Take only vehicles with bounding_box attribute set
                    if hasattr(vehicle, "bounding_box"):
                        
                        # Get the location of the target
                        target_location = vehicle.transform.location

                        # Check visibility of the target
                        if self.check_visibility(ego_vehicle_location, target_location):
                            vehicle_obj = self._get_vehicle_from_environment_objects(vehicle, object_value)
                            ros_objects.objects.append(vehicle_obj)

        self.object_publisher.publish(ros_objects)