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
import re

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
        self.georeference_substitution = (self.node.parameters['georeference_substitution'])
        self.grid_convergence_override = self.node.parameters['grid_convergence']
        self.projection_string = ""

        self.world_frame = "map"
        self.world_x = 0.0
        self.world_y = 0.0
        self.q_grid_convergence = quaternion_from_euler(0, 0, 0)
        self.transform_utm_to_carla = None
        self.zone = None
        self.northp = True

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

            opendrive = self.carla_map.to_opendrive()
            root = ET.fromstring(opendrive)

            # replace georeference inside the OpenDRIVE xml string
            geo_reference = root.find(".//geoReference")
            if geo_reference is not None and geo_reference.text is not None and \
                    self.georeference_substitution is not None and len(self.georeference_substitution) != 0:
                geo_reference.text = self.georeference_substitution
                opendrive = ET.tostring(root, encoding="unicode", method="xml")
                self.node.loginfo("Applied georeference_substitution.")

            for header in root.findall('header'):
                for geo in header.findall('geoReference'):
                    self.projection_string = (geo.text or "").strip()
                    if not self.projection_string:
                        continue
                    self.node.loginfo("geoReference projection string: {}".format(self.projection_string))

                    proj_xodr = pyproj.Proj(self.projection_string)

                    # consider offset or +x_0/+y_0 in projection string to find projection origin
                    off = header.find('offset')
                    if off is not None:
                        ox = float(off.get('x', 0.0))
                        oy = float(off.get('y', 0.0))
                        self.node.loginfo("Using OpenDRIVE offset: x={} y={}".format(ox, oy))
                    else:
                        sl = self.projection_string.lower()
                        ox = self._get_float_param(sl, "x_0")
                        oy = self._get_float_param(sl, "y_0")
                        ox = ox if ox is not None else 0.0
                        oy = oy if oy is not None else 0.0
                        self.node.loginfo("No OpenDRIVE offset. Using +x_0/+y_0 fallback: x={} y={}".format(
                            ox, oy))

                    # find lat/lon of projection origin and map origin
                    # (0, 0) is the OpenDRIVE map origin in projected coordinates,
                    # not the projection's false-origin (+x_0/+y_0).
                    lon_map_origin, lat_map_origin = proj_xodr(0, 0, inverse=True)
                    lon_proj_origin, lat_proj_origin = proj_xodr(ox, oy, inverse=True)

                    # derive UTM zone and world frame name from projection origin
                    self.zone = int(math.floor((lon_proj_origin + 180.0 + 1e-12) / 6.0) + 1)
                    self.zone = max(1, min(60, self.zone))
                    self.northp = (lat_proj_origin >= 0.0)
                    self.world_frame = "utm_{}{}".format(self.zone, "N" if self.northp else "S")
                    self.node.loginfo("Derived world frame '{}' from lon_proj_origin={} lat_proj_origin={}".format(
                        self.world_frame, lon_proj_origin, lat_proj_origin))

                    # set up projection for world origin
                    if self.northp:
                        p = pyproj.Proj(proj='utm', zone=self.zone, ellps='WGS84', preserve_units=False)
                    else:
                        p = pyproj.Proj(proj='utm', zone=self.zone, south=True, ellps='WGS84',
                                        preserve_units=False)

                    # calculate world origin in CARLA coordinates
                    self.world_x, self.world_y = p(lon_map_origin, lat_map_origin)
                    self.node.loginfo("World transform set: frame='{}' translation=({}, {}, 0.0)".format(
                        self.world_frame, self.world_x, self.world_y))

                    apply_gc, reason = self._check_grid_convergence(self.projection_string)
                    self.node.loginfo("Grid convergence check: {} ({})".format(reason, apply_gc))
                    if apply_gc:
                        center_lon = 6.0 * float(self.zone) - 183.0
                        grid_convergence = math.atan(
                            math.tan(lon_proj_origin * math.pi / 180.0 - center_lon * math.pi / 180.0) *
                            math.sin(lat_proj_origin * math.pi / 180.0))
                        self.q_grid_convergence = quaternion_from_euler(0, 0, grid_convergence)
                    else:
                        self.q_grid_convergence = quaternion_from_euler(0, 0, 0)

                    self.world_set = True
                    break
                if self.world_set:
                    break

            # publish world info
            open_drive_msg = CarlaWorldInfo()
            open_drive_msg.map_name = self.carla_map.name
            open_drive_msg.opendrive = opendrive
            self.world_info_publisher.publish(open_drive_msg)
            self.map_published = True

            # if no geo reference found in OpenDRIVE, align 'carla_map' frame with 'map' frame
            if not self.world_set:
                self.world_frame = "map"
                self.world_x = 0.0
                self.world_y = 0.0
                self.q_grid_convergence = quaternion_from_euler(0, 0, 0)
                self.node.logwarn("No valid geoReference found. Falling back to world frame '{}'.".format(
                    self.world_frame))

        # create transform message
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

        self.transform_utm_to_carla = t

        # publish transform message
        self._tf_broadcaster.sendTransform(t)
        self.node.logdebug("Published transform {} -> {} at x={} y={}".format(
            self.world_frame, self.map_frame, self.world_x, self.world_y))



    @staticmethod
    def _get_float_param(s: str, key: str):
        m = re.search(rf"\+{re.escape(key)}=([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)", s)
        return float(m.group(1)) if m else None


    def _check_grid_convergence(self, proj_string: str):

        override = self.grid_convergence_override
        if isinstance(override, str):
            override = override.strip().lower()
            override = {"true": True, "false": False}.get(override, None)
        if override is not None:
            return bool(override), "override -> {}".format(override)

        s = (proj_string or "").strip()
        sl = s.lower()

        if "+proj=utm" in sl:
            return False, "utm -> not apply grid convergence"

        if "+proj=tmerc" in sl:
            k  = self._get_float_param(sl, "k")
            x0 = self._get_float_param(sl, "x_0")
            y0 = self._get_float_param(sl, "y_0")

            # UTM-like TM
            if k is not None and x0 is not None:
                if abs(k - 0.9996) < 1e-4 and abs(x0 - 500000.0) < 5.0:
                    return False, "tmerc with UTM params -> not apply grid convergence"

            #  Local custom coordinate system
            if k is not None and x0 is not None and y0 is not None:
                if abs(k - 1.0) < 1e-6 and abs(x0) < 1e-6 and abs(y0) < 1e-6:
                    return True, "tmerc with k=1, x0=y0=0 -> apply grid convergence"

            return True, "tmerc default -> apply grid convergence"

        return False, "unknown proj -> default NO"
