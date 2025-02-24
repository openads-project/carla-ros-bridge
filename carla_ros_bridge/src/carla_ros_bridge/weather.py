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

from ros_compatibility.qos import QoSProfile, DurabilityPolicy
from carla_msgs.msg import WeatherParameters


class Weather(object):

    """
    Publish the weather
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
        self.world = carla_world

        self.weather_published = False
        self.weather_publisher = node.new_publisher(
            WeatherParameters,
            "/carla/weather",
            qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))


    def destroy(self):
        """
        Function (override) to destroy this object.

        Remove reference to carla.Map object.
        Finally forward call to super class.

        :return:
        """
        self.node.destroy_publisher(self.weather_publisher)
        self.weather = None


    def update(self, frame, timestamp):
        """
        Function (override) to update this object.

        :return:
        """
        weather = WeatherParameters()
        weather = self.world.get_weather()
        print(weather)

        if not self.weather_published:

            self.weather_publisher.publish(weather)
            self.weather_published = True
