#!/usr/bin/env python
#
# Copyright (c) 2026
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Utilities for CARLA map queries.
"""

import carla


def get_road_altitude(carla_world, location, loginfo=None, default=None):
    """
    Get the road altitude at a given CARLA location.

    Returns 'default' when the map has no road below the location.
    """
    carla_map = carla_world.get_map()
    waypoint = carla_map.get_waypoint(
        location, project_to_road=True, lane_type=carla.LaneType.Driving)

    if waypoint is not None:
        return waypoint.transform.location.z

    if loginfo:
        loginfo("Could not find waypoint for position x={}, y={}".format(
            location.x, location.y))
    return default


def lift_if_below_road(carla_world, transform, z_offset=2.0, loginfo=None):
    """
    Lift a transform above the road when its current altitude is below the map.
    """
    # without a road below it there is nothing to lift the transform above, so
    # compare it against itself and leave it where it is
    road_altitude = get_road_altitude(
        carla_world, transform.location, loginfo, default=transform.location.z)
    if transform.location.z - road_altitude < 0:
        transform.location.z = road_altitude + z_offset
        return True
    return False
