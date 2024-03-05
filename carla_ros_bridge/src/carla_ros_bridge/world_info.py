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
import pyproj
import math

ROS_VERSION = get_ros_version()

if ROS_VERSION == 1:
    from tf.transformations import quaternion_from_euler
elif ROS_VERSION == 2:
    from tf_transformations import quaternion_from_euler
else:
    raise NotImplementedError("Unsupported ROS version")


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
        self.map_frame = "carla_map"
        self.world_set = False

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
                    projection_string = geo.text

                    # get lat and lon from projection string
                    proj_xodr = pyproj.Proj(projparams=projection_string)
                    lon, lat = proj_xodr(0, 0, inverse=True)

                    # derive utm zone and set frame id
                    if lat >= 0.0: northp = True
                    else: northp = False
                    zone = int(math.floor((lon + 180.0)/6.0) + 1)
                    if northp:
                        p = pyproj.Proj(proj='utm',zone=zone,ellps='WGS84', preserve_units=False)
                        self.world_frame = "utm_" + str(zone) + "N"
                    else:
                        p = pyproj.Proj(proj='utm',zone=zone, south=True, ellps='WGS84', preserve_units=False)
                        self.world_frame = "utm_" + str(zone) + "S"
                    
                    # calculate grid convergence
                    center_lon = 6.0 * float(zone) - 183.0
                    grid_convergence = math.atan(math.tan(lon * math.pi / 180.0 - center_lon * math.pi / 180.0) * math.sin(lat * math.pi / 180.0))
                    self.q_grid_convergence = quaternion_from_euler(0, 0, grid_convergence)

                    print("Publishing transform from {} to {}".format(self.world_frame, self.map_frame))

                    self.world_x, self.world_y = p(lon,lat)
                    self.world_set = True

        # publish transform 
        if self.world_set:

            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = roscomp.ros_timestamp(sec=timestamp + self.node.parameters["start_unix_time_stamp"], from_sec=True)
            t.header.frame_id = self.world_frame
            t.child_frame_id = self.map_frame

            t.transform.translation.x = self.world_x
            t.transform.translation.y = self.world_y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = self.q_grid_convergence[0]
            t.transform.rotation.y = self.q_grid_convergence[1]
            t.transform.rotation.z = self.q_grid_convergence[2]
            t.transform.rotation.w = self.q_grid_convergence[3]

            self._tf_broadcaster.sendTransform(t)