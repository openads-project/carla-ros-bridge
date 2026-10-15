#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla vehicles
"""

import carla_common.transforms as trans

from carla_ros_bridge.traffic_participant import TrafficParticipant

from derived_object_msgs.msg import Object
from std_msgs.msg import ColorRGBA


class Vehicle(TrafficParticipant):

    """
    Actor implementation details for vehicles
    """

    # values of the legacy 'object_type' blueprint attribute
    OBJECT_TYPE_CLASSIFICATION = {
        'car': Object.CLASSIFICATION_CAR,
        'bike': Object.CLASSIFICATION_BIKE,
        'motorcycle': Object.CLASSIFICATION_MOTORCYCLE,
        'truck': Object.CLASSIFICATION_TRUCK,
        'other': Object.CLASSIFICATION_OTHER_VEHICLE,
    }

    # values of CARLA's 'base_type' blueprint attribute
    BASE_TYPE_CLASSIFICATION = {
        'car': Object.CLASSIFICATION_CAR,
        'van': Object.CLASSIFICATION_CAR,
        'truck': Object.CLASSIFICATION_TRUCK,
        'bus': Object.CLASSIFICATION_OTHER_VEHICLE,
        'motorcycle': Object.CLASSIFICATION_MOTORCYCLE,
        'bicycle': Object.CLASSIFICATION_BIKE,
    }

    def __init__(self, uid, name, parent, node, carla_actor):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        :param carla_actor: carla vehicle actor object
        :type carla_actor: carla.Vehicle
        """
        # an explicit 'object_type' takes precedence; CARLA ships it empty, so
        # fall back to 'base_type', which CARLA sets for every vehicle blueprint
        object_type = carla_actor.attributes.get('object_type', '')
        base_type = carla_actor.attributes.get('base_type', '')
        self.classification = self.OBJECT_TYPE_CLASSIFICATION.get(
            object_type,
            self.BASE_TYPE_CLASSIFICATION.get(base_type, Object.CLASSIFICATION_CAR))

        super(Vehicle, self).__init__(uid=uid,
                                      name=name,
                                      parent=parent,
                                      node=node,
                                      carla_actor=carla_actor)

    def get_marker_color(self):  # pylint: disable=no-self-use
        """
        Function (override) to return the color for marker messages.

        :return: the color used by a vehicle marker
        :rtpye : std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 255.0
        color.g = 0.0
        color.b = 0.0
        return color

    def get_marker_pose(self):
        """
        Function to return the pose for vehicles.

        :return: the pose of the vehicle
        :rtype: geometry_msgs.msg.Pose
        """
        # Moving pivot point from the bottom (CARLA) to the center (ROS) of the bounding box.
        extent = self.carla_actor.bounding_box.extent
        marker_transform = self.carla_actor.get_transform()
        marker_transform.location += marker_transform.get_up_vector() * extent.z
        return trans.carla_transform_to_ros_pose(marker_transform)

    def get_classification(self):
        """
        Function (override) to get classification
        :return:
        """
        return self.classification
