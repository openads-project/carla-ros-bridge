#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla gnsss
"""

import pyproj

from carla_ros_bridge.sensor import Sensor

from sensor_msgs.msg import NavSatFix


class Gnss(Sensor):

    """
    Actor implementation details for gnss sensor
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the relative spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(Gnss, self).__init__(uid=uid,
                                   name=name,
                                   parent=parent,
                                   relative_spawn_pose=relative_spawn_pose,
                                   node=node,
                                   carla_actor=carla_actor,
                                   synchronous_mode=synchronous_mode)

        self.gnss_publisher = node.new_publisher(NavSatFix,
                                                 self.get_topic_prefix(),
                                                 qos_profile=10)
        self._projection_string = None
        self._projection = None
        self.listen()

    def destroy(self):
        super(Gnss, self).destroy()
        self.node.destroy_publisher(self.gnss_publisher)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_gnss_measurement):
        """
        Function to transform a received gnss event into a ROS NavSatFix message

        :param carla_gnss_measurement: carla gnss measurement object
        :type carla_gnss_measurement: carla.GnssMeasurement
        """
        navsatfix_msg = NavSatFix()
        navsatfix_msg.header = self.get_msg_header(timestamp=carla_gnss_measurement.timestamp)
        if self.node.parameters.get('georeference_substitution'):
            latlon = self._update_lat_lon(carla_gnss_measurement)
            if latlon is None:
                raise RuntimeError(
                    "georeference_substitution is active, but GNSS lat/lon could not be derived from position.")
            navsatfix_msg.latitude, navsatfix_msg.longitude = latlon
        else:
            navsatfix_msg.latitude = carla_gnss_measurement.latitude
            navsatfix_msg.longitude = carla_gnss_measurement.longitude

        if self.node.parameters['ignore_altitude']:
            navsatfix_msg.altitude = 0.0
        else:
            navsatfix_msg.altitude = carla_gnss_measurement.altitude

        self.gnss_publisher.publish(navsatfix_msg)

    def _update_lat_lon(self, carla_gnss_measurement):
        """
        Convert the measurement position to latitude/longitude using the active world georeference.

        This path is only used when a georeference substitution is configured. It ensures GNSS
        output follows the substituted projection instead of CARLA's internal GNSS conversion.

        :param carla_gnss_measurement: carla gnss measurement object
        :type carla_gnss_measurement: carla.GnssMeasurement
        :return: latitude/longitude if conversion is available
        :rtype: tuple(float, float) or None
        """

        world_info = getattr(self.node, 'world_info', None)
        projection_string = ""
        if world_info is not None:
            projection_string = (world_info.projection_string or "").strip()
        if not projection_string:
            return None

        if projection_string != self._projection_string:
            self._projection_string = projection_string
            try:
                self._projection = pyproj.Proj(projparams=projection_string)
            except RuntimeError as error:
                self.node.logwarn("Failed to parse georeference projection '{}': {}".format(
                    projection_string, error))
                self._projection = None
                return None

        if self._projection is None:
            return None

        try:
            location = carla_gnss_measurement.transform.location
            lon, lat = self._projection(location.x, -location.y, inverse=True)
            print("GNSS debug: x={}, y={}, lat={}, lon={}".format(location.x, location.y, lat, lon))
        except (AttributeError, RuntimeError) as error:
            self.node.logwarn(
                "Failed to derive GNSS fix from sensor position: {}"
                .format(error))
            return None

        return lat, lon
