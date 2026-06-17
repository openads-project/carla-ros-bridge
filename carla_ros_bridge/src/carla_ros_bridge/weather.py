#!/usr/bin/env python
#
# Copyright (c) Institute for Automotive Engineering (ika), RWTH Aachen University
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Class to provide weather information
"""

from ros_compatibility.qos import QoSProfile, DurabilityPolicy
from carla_msgs.msg import CarlaWeatherParameters

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

        node.new_timer(1.0, self.update)
        self.weather_publisher = node.new_publisher(
            CarlaWeatherParameters,
            "/carla/weather",
            qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))


    def destroy(self):
        """
        Function (override) to destroy this object.

        Finally forward call to super class.

        :return:
        """
        self.node.destroy_publisher(self.weather_publisher)


    def update(self):
        """
        Function (override) to update this object.

        :return:
        """
        weather = CarlaWeatherParameters()

        carla_weather = self.world.get_weather()
        weather.cloudiness = carla_weather.cloudiness
        weather.precipitation = carla_weather.precipitation
        weather.precipitation_deposits = carla_weather.precipitation_deposits
        weather.wind_intensity = carla_weather.wind_intensity
        weather.fog_density = carla_weather.fog_density
        weather.fog_distance = carla_weather.fog_distance
        weather.wetness = carla_weather.wetness
        weather.sun_azimuth_angle = carla_weather.sun_azimuth_angle
        weather.sun_altitude_angle = carla_weather.sun_altitude_angle


        self.weather_publisher.publish(weather)
