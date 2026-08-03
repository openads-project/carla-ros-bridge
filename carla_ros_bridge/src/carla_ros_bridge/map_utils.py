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


def get_road_altitude(carla_world, location, loginfo=None):
    """
    Get the road altitude at a given CARLA location.
    """
    carla_map = carla_world.get_map()
    waypoint = carla_map.get_waypoint(
        location, project_to_road=True, lane_type=carla.LaneType.Driving)

    if waypoint is not None:
        return waypoint.transform.location.z

    if loginfo:
        loginfo("Could not find waypoint for position x={}, y={}".format(
            location.x, location.y))
    return location.z


def get_ground_altitude(carla_world, location, probe_height=100.0, loginfo=None):
    """
    Get the ground altitude at a given CARLA location.

    Casts a ray straight down from probe_height above the location, so it works
    anywhere on the map. get_road_altitude() is only used as a fallback: it
    snaps to the nearest driving lane, which can be far off for a position on a
    sidewalk or a green area. For edge cases, like tunnels, manual tuning is needed.
    """
    probe = carla.Location(location.x, location.y, location.z + probe_height)
    hit = carla_world.ground_projection(probe, 2.0 * probe_height)

    if hit is not None:
        return hit.location.z

    if loginfo:
        loginfo("No ground hit below x={}, y={}, falling back to road altitude".format(
            location.x, location.y))
    return get_road_altitude(carla_world, location, loginfo)


def lift_if_below_road(carla_world, transform, z_offset=2.0, loginfo=None):
    """
    Lift a transform above the road when its current altitude is below the map.
    """
    road_altitude = get_road_altitude(carla_world, transform.location, loginfo)
    if transform.location.z - road_altitude < 0:
        transform.location.z = road_altitude + z_offset
        return True
    return False
