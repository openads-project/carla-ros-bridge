#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Class to handle the carla map
"""

import tf2_ros
import geometry_msgs.msg
import ros_compatibility as roscomp
from ros_compatibility.core import get_ros_version
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_msgs.msg import CarlaWorldInfo

import xml.etree.ElementTree as ET
from pyproj import Proj

ROS_VERSION = get_ros_version()


class WorldInfo(object):

    """
    Publish the map
    """

    def __init__(self, carla_world, node):
        """
        Constructor

        :param carla_world: carla world object
        :type carla_world: carla.World
        :param node: node-handle
        :type node: CompatibleNode
        """
        self.node = node
        self.carla_map = carla_world.get_map()

        self.map_published = False

        self.world_info_publisher = node.new_publisher(
            CarlaWorldInfo,
            "/carla/world_info",
            qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        if ROS_VERSION == 1:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster()       # ROS 1
        elif ROS_VERSION == 2:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(node)   # ROS 2


    def destroy(self):
        """
        Function (override) to destroy this object.

        Remove reference to carla.Map object.
        Finally forward call to super class.

        :return:
        """
        self.node.destroy_publisher(self.world_info_publisher)
        self.carla_map = None

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.

        :return:
        """
        if not self.map_published:
            open_drive_msg = CarlaWorldInfo()
            open_drive_msg.map_name = self.carla_map.name
            open_drive_msg.opendrive = self.carla_map.to_opendrive()
            self.world_info_publisher.publish(open_drive_msg)
            self.map_published = True

            # extract transform 
            root = ET.fromstring(open_drive_msg.opendrive)

            for header in root.findall('header'):
                for geo in header.findall('geoReference'):
                    proj = geo.text

                    p = Proj(proj='utm',zone=10,ellps='WGS84', preserve_units=False)

                    self.world_x, self.world_y = p(0,0)
        
        # publish transform 
        if self.world_x and self.world_y:

            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = roscomp.ros_timestamp(sec=timestamp, from_sec=True)
            t.header.frame_id = "world"
            t.child_frame_id = "carla_map"

            t.transform.translation.x = self.world_x
            t.transform.translation.y = self.world_y
            t.transform.rotation.w = 1.0

            self._tf_broadcaster.sendTransform(t)