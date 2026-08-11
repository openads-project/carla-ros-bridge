#!/usr/bin/env python
#
# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
Welcome to CARLA ROS manual control.

If Xbox controller is connected, use the following trigger for control.

    right trigger (RT)          : throttle
    left trigger (LT)           : brake
    left joystick (left right)  : steer left/right
    left button (LB)            : toggle reverse
    right button (RB)           : hand-brake
    A button                    : toggle autopilot
    Y button                    : toggle manual control
    
    view button                 : toggle HUD
    Share button                : toggle help
    Xbox button                 : quit


If no Xbox controller is connected, use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    AD           : steer
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    B            : toggle manual control

    F1           : toggle HUD
    H/?          : toggle help
    ESC          : quit
"""

from __future__ import print_function

import datetime
import math
from threading import Thread

import numpy
import os
from transforms3d.euler import quat2euler
import cv2
try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import KMOD_SHIFT
    from pygame.locals import K_COMMA
    from pygame.locals import K_DOWN
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_F1
    from pygame.locals import K_LEFT
    from pygame.locals import K_PERIOD
    from pygame.locals import K_RIGHT
    from pygame.locals import K_SLASH
    from pygame.locals import K_SPACE
    from pygame.locals import K_UP
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_h
    from pygame.locals import K_m
    from pygame.locals import K_p
    from pygame.locals import K_q
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_b
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from carla_msgs.msg import CarlaStatus
from carla_msgs.msg import CarlaEgoVehicleInfo
from carla_msgs.msg import CarlaEgoVehicleStatus
from carla_msgs.msg import CarlaEgoVehicleControl
from carla_msgs.msg import CarlaLaneInvasionEvent
from carla_msgs.msg import CarlaCollisionEvent
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool

# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================

fast_qos = QoSProfile(depth=10)
fast_latched_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)

class ManualControl(CompatibleNode):
    """
    Handle the rendering
    """

    def __init__(self, resolution, joystick):
        super(ManualControl, self).__init__("ManualControl")
        self._surface = None
        self.role_name = self.get_param("role_name", "ego_vehicle")
        self.wireless_controller = self.get_param("wireless_controller", False)
        self.joystick_available = True if joystick else False

        self.hud = HUD(self.role_name, resolution['width'], resolution['height'], self)
        self.vehicle_control_manual_override = False
        self.autopilot_enabled = False
        self.prio_publish = False
        if joystick:
            self.xbox_controller = XboxControl(self.role_name, self.hud, self, joystick)
        else:
            self.xbox_controller = None
        self.controller = KeyboardControl(self.role_name, self.hud, self)

        self.image_subscriber = self.new_subscription(
            Image, "/carla/{}/rgb_view/image".format(self.role_name),
            self.on_view_image, qos_profile=10)
        self.compressed_image_subscriber = self.new_subscription(
            CompressedImage, "/carla/{}/rgb_view/image/compressed".format(self.role_name),
            self.on_view_compressed_image, qos_profile=10)

        self.collision_subscriber = self.new_subscription(
            CarlaCollisionEvent, "/carla/{}/collision".format(self.role_name),
            self.on_collision, qos_profile=10)

        self.lane_invasion_subscriber = self.new_subscription(
            CarlaLaneInvasionEvent, "/carla/{}/lane_invasion".format(self.role_name),
            self.on_lane_invasion, qos_profile=10)

        self.carla_status_subscriber = self.new_subscription(
            CarlaStatus,
            "/carla/status",
            self.on_new_carla_frame,
            qos_profile=10)

        self.vehicle_control_publisher = self.new_publisher(
            CarlaEgoVehicleControl,
            "/carla/{}/vehicle_control_cmd_manual".format(self.role_name),
            qos_profile=fast_qos)

        self.vehicle_control_manual_override_publisher = self.new_publisher(
            Bool,
            "/carla/{}/vehicle_control_manual_override".format(self.role_name),
            qos_profile=fast_latched_qos)

        self.auto_pilot_enable_publisher = self.new_publisher(
            Bool,
            "/carla/{}/enable_autopilot".format(self.role_name),
            qos_profile=fast_qos)

        self.set_autopilot(self.autopilot_enabled)

        self.set_vehicle_control_manual_override(
            self.vehicle_control_manual_override)  # disable manual override

    def set_vehicle_control_manual_override(self, enable):
        """
        Set the manual control override
        """
        self.hud.notification('Set vehicle control manual override to: {}'.format(enable))
        self.vehicle_control_manual_override_publisher.publish((Bool(data=enable)))

    def set_autopilot(self, enable):
        """
        enable/disable the autopilot
        """
        self.auto_pilot_enable_publisher.publish(Bool(data=enable))

    def on_new_carla_frame(self, data):
        """
        callback on new frame

        As CARLA only processes one vehicle control command per tick,
        send the current from within here (once per frame)
        """
        control = self.controller._control
        if self.xbox_controller and control.throttle == 0 and control.brake == 0 and control.steer == 0 and not control.hand_brake:
            control = self.xbox_controller._control

        input = ((control.throttle > 0) or (control.brake > 0) or (control.steer != 0) or (control.hand_brake))

        # Activate vehicle_control_manual_override if input detected
        if input and not self.vehicle_control_manual_override:
            self.vehicle_control_manual_override = True
            self.hud.notification('Vehicle control manual override activated')
            self.set_vehicle_control_manual_override(True)

        # Disable autopilot if controller input is detected
        if input and self.autopilot_enabled:
            self.autopilot_enabled = False
            self.set_autopilot(False)
            self.hud.notification('Autopilot Off')

        # Send vehicle control command
        if not self.autopilot_enabled and self.vehicle_control_manual_override:
            try:
                self.vehicle_control_publisher.publish(control)
            except Exception as error:
                self.node.logwarn("Could not send vehicle control: {}".format(error))

    def on_collision(self, data):
        """
        Callback on collision event
        """
        intensity = math.sqrt(data.normal_impulse.x**2 +
                              data.normal_impulse.y**2 + data.normal_impulse.z**2)
        self.hud.notification('Collision with {} (impulse {})'.format(
            data.other_actor_id, intensity))

    def on_lane_invasion(self, data):
        """
        Callback on lane invasion event
        """
        text = []
        for marking in data.crossed_lane_markings:
            if marking is CarlaLaneInvasionEvent.LANE_MARKING_OTHER:
                text.append("Other")
            elif marking is CarlaLaneInvasionEvent.LANE_MARKING_BROKEN:
                text.append("Broken")
            elif marking is CarlaLaneInvasionEvent.LANE_MARKING_SOLID:
                text.append("Solid")
            else:
                text.append("Unknown ")
        self.hud.notification('Crossed line %s' % ' and '.join(text))

    def on_view_image(self, image):
        """
        Callback when receiving a camera image
        """
        array = numpy.frombuffer(image.data, dtype=numpy.dtype("uint8"))
        array = numpy.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    def on_view_compressed_image(self, image):
        """
        Callback when receiving a compressed camera image
        """
        np_arr = numpy.frombuffer(image.data, dtype=numpy.uint8)
        array = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if array is None:
            self.node.logwarn("Could not decode compressed image")
            return
        array = cv2.cvtColor(array, cv2.COLOR_BGR2RGB)
        self._surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    def render(self, game_clock, display):
        """
        render the current image
        """
        events = pygame.event.get()
        if self.controller.parse_events(game_clock, events):
            return True
        if self.xbox_controller and self.xbox_controller.parse_events(events):
            return True
        self.hud.tick(game_clock)

        if self._surface is not None:
            display.blit(self._surface, (0, 0))
        self.hud.render(display)

# ==============================================================================
# -- KeyboardControl -----------------------------------------------------------
# ==============================================================================


class KeyboardControl(object):
    """
    Handle input events
    """

    def __init__(self, role_name, hud, node):
        self.role_name = role_name
        self.hud = hud
        self.node = node

        self._control = CarlaEgoVehicleControl()
        self._steer_cache = 0.0

    # pylint: disable=too-many-branches
    def parse_events(self, clock, events):
        """
        parse an input event
        """
        for event in events:
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True
                elif event.key == K_F1:
                    self.hud.toggle_info()
                elif event.key == K_h or (event.key == K_SLASH and
                                          pygame.key.get_mods() & KMOD_SHIFT):
                    self.hud.help.toggle()
                elif event.key == K_b:
                    self.node.vehicle_control_manual_override = not self.node.vehicle_control_manual_override
                    self.node.set_vehicle_control_manual_override(self.node.vehicle_control_manual_override)
                if event.key == K_q:
                    self._control.gear = 1 if self._control.reverse else -1
                elif event.key == K_m:
                    self._control.manual_gear_shift = not self._control.manual_gear_shift
                    self.hud.notification(
                        '%s Transmission' %
                        ('Manual' if self._control.manual_gear_shift else 'Automatic'))
                elif self._control.manual_gear_shift and event.key == K_COMMA:
                    self._control.gear = max(-1, self._control.gear - 1)
                elif self._control.manual_gear_shift and event.key == K_PERIOD:
                    self._control.gear = self._control.gear + 1
                elif event.key == K_p:
                    self.node.autopilot_enabled = not self.node.autopilot_enabled
                    self.node.set_autopilot(self.node.autopilot_enabled)
                    self.hud.notification('Autopilot %s' %
                                          ('On' if self.node.autopilot_enabled else 'Off'))
        
        self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time())
        self._control.reverse = self._control.gear < 0

    def _parse_vehicle_keys(self, keys, milliseconds):
        """
        parse key events
        """
        self._control.throttle = 1.0 if keys[K_UP] or keys[K_w] else 0.0
        steer_increment = 5e-4 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self._steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self._steer_cache += steer_increment
        else:
            self._steer_cache = 0.0
        self._steer_cache = min(0.7, max(-0.7, self._steer_cache))
        self._control.steer = round(self._steer_cache, 1)
        self._control.brake = 1.0 if keys[K_DOWN] or keys[K_s] else 0.0
        self._control.hand_brake = bool(keys[K_SPACE])

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


# ==============================================================================
# -- XboxControl ---------------------------------------------------------------
# ==============================================================================


class XboxControl(object):
    """
    Handle input events
    """

    def __init__(self, role_name, hud, node, joystick):
        self.role_name = role_name
        self.hud = hud
        self.node = node
        self.joystick = joystick
        
        self._control = CarlaEgoVehicleControl()
        self._steer_cache = 0.0
        self._throttle_cache = 0.0
        self._brake_cache = 0.0
        self._wireless = node.wireless_controller
        
        self._controller_layout = {
            "steer":            [0,   0],
            "throttle":         [5,   4],
            "brake":            [2,   5],
            "reverse":          [4,   6],
            "hand_brake":       [5,   7],
            "toggle_HUD":       [6,  10],
            "enable_autopilot": [0,   0],
            "manual_control":   [3,   4],
            "quit_shortcut":    [8,  12],
            "Help":             [11, 15]
        }

    def change_movement_direction(self, v_res):
        if v_res < 2:
            return True
        else:
            self.hud.notification('Gear change only possible while stationary!')
            return False
        pass

    def parse_events(self, events):
        v_res = 3.6 * self.hud.vehicle_status.velocity
        for event in events:
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.JOYBUTTONUP:
                if self._is_quit_shortcut(event.button):
                    return True
                elif event.button == self._controller_layout["toggle_HUD"][self._wireless]:
                    self.hud.toggle_info()
                elif event.button == self._controller_layout["Help"][self._wireless]:
                    self.hud.help.toggle()
                elif event.button == self._controller_layout["manual_control"][self._wireless]:
                    self.node.vehicle_control_manual_override = not self.node.vehicle_control_manual_override
                    self.node.set_vehicle_control_manual_override(self.node.vehicle_control_manual_override)
                elif event.button == self._controller_layout["enable_autopilot"][self._wireless]:
                    self.node.autopilot_enabled = not self.node.autopilot_enabled
                    self.node.set_autopilot(self.node.autopilot_enabled)
                    self.hud.notification('Autopilot %s' %
                                         ('On' if self.node.autopilot_enabled else 'Off'))
                elif event.button == self._controller_layout["reverse"][self._wireless] and self.change_movement_direction(v_res):
                    self._control.gear = 1 if self._control.reverse else -1
        
        self._parse_vehicle_keys(v_res)
        self._control.reverse = self._control.gear < 0

    @staticmethod
    def steering_charakteristic(joystick_position, velocity):
        a = 1.2346  # Stretch parameter
        b = 0.1     # Shift parameter
        c = 2       # Exponent
        if abs(joystick_position) > b and velocity < 10:
            return math.copysign(1, joystick_position) * a * (abs(joystick_position) - b)**c
        elif abs(joystick_position) > b:
            return math.copysign(1, joystick_position) * a * math.e**(-velocity/100) * (abs(joystick_position) - b)**c
        else: return 0.0

    @staticmethod
    def brake_charakteristic(joystick_position):
        if joystick_position > - 0.75:
            return 4/7 * joystick_position + 3/7
        else: return 0.0

    @staticmethod
    def throttle_charakteristic(joystick_position):
        if joystick_position > - 0.5:
            return 2/3 * joystick_position + 1/3
        else: return 0.0

    def _parse_vehicle_keys(self, velocity):
        throttle_axis = self._controller_layout["throttle"][self._wireless]
        # Throttle control
        if self.joystick.get_axis(throttle_axis):
            self._throttle_cache = self.throttle_charakteristic(self.joystick.get_axis(throttle_axis))
        else:
            self._throttle_cache = 0.0
        self._control.throttle = round(self._throttle_cache, 3)

        # Brake control
        brake_axis = self._controller_layout["brake"][self._wireless]
        if self.joystick.get_axis(brake_axis):
            self._brake_cache = self.brake_charakteristic(self.joystick.get_axis(brake_axis))
        else:
            self._brake_cache = 0.0
        self._control.brake = round(self._brake_cache, 3)

        # Steer control
        steer_axis = self._controller_layout["steer"][self._wireless]
        if self.joystick.get_axis(steer_axis):
            self._steer_cache = self.steering_charakteristic(self.joystick.get_axis(steer_axis), velocity)
        else:
            self._steer_cache = 0.0
        self._control.steer = round(self._steer_cache, 3)

        # Set hand brake
        if self.joystick.get_button(self._controller_layout["hand_brake"][self._wireless]):
            self._control.hand_brake = True
        else: self._control.hand_brake = False

    def _is_quit_shortcut(self, button):
        return (button == self._controller_layout["quit_shortcut"][self._wireless])


# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    """
    Handle the info display
    """

    def __init__(self, role_name, width, height, node):
        self.role_name = role_name
        self.dim = (width, height)
        self.node = node
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        fonts = [x for x in pygame.font.get_fonts() if 'mono' in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 14), width, height)
        self._show_info = True
        self._info_text = []
        self.vehicle_status = CarlaEgoVehicleStatus()

        self.vehicle_status_subscriber = node.new_subscription(
            CarlaEgoVehicleStatus, "/carla/{}/vehicle_status".format(self.role_name),
            self.vehicle_status_updated, qos_profile=10)

        self.vehicle_info = CarlaEgoVehicleInfo()
        self.vehicle_info_subscriber = node.new_subscription(
            CarlaEgoVehicleInfo,
            "/carla/{}/vehicle_info".format(self.role_name),
            self.vehicle_info_updated, 
            qos_profile=QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.x, self.y, self.z = 0, 0, 0
        self.yaw = 0
        self.latitude = 0
        self.longitude = 0
        self.manual_control = False

        self.gnss_subscriber = node.new_subscription(
            NavSatFix,
            "/carla/{}/gnss".format(self.role_name),
            self.gnss_updated,
            qos_profile=10)

        self.odometry_subscriber = node.new_subscription(
            Odometry,
            "/carla/{}/odometry".format(self.role_name),
            self.odometry_updated,
            qos_profile=10)

        self.manual_control_subscriber = node.new_subscription(
            Bool,
            "/carla/{}/vehicle_control_manual_override".format(self.role_name),
            self.manual_control_override_updated,
            qos_profile=10)

        self.carla_status = CarlaStatus()
        self.status_subscriber = node.new_subscription(
            CarlaStatus,
            "/carla/status",
            self.carla_status_updated,
            qos_profile=10)

    def tick(self, clock):
        """
        tick method
        """
        self._notifications.tick(clock)

    def carla_status_updated(self, data):
        """
        Callback on carla status
        """
        self.carla_status = data
        self.update_info_text()

    def manual_control_override_updated(self, data):
        """
        Callback on vehicle status updates
        """
        self.manual_control = data.data
        self.update_info_text()

    def vehicle_status_updated(self, vehicle_status):
        """
        Callback on vehicle status updates
        """
        self.vehicle_status = vehicle_status
        self.update_info_text()

    def vehicle_info_updated(self, vehicle_info):
        """
        Callback on vehicle info updates
        """
        self.vehicle_info = vehicle_info
        self.update_info_text()

    def gnss_updated(self, data):
        """
        Callback on gnss position updates
        """
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.update_info_text()

    def odometry_updated(self, data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
        _, _, yaw = quat2euler(
            [data.pose.pose.orientation.w,
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z])
        self.yaw = math.degrees(yaw)
        self.update_info_text()

    def update_info_text(self):
        """
        update the displayed info text
        """
        if not self._show_info:
            return

        x, y, z = self.x, self.y, self.z
        yaw = self.yaw

        heading = 'N' if abs(yaw) < 89.5 else ''
        heading += 'S' if abs(yaw) > 90.5 else ''
        heading += 'E' if 179.5 > yaw > 0.5 else ''
        heading += 'W' if -0.5 > yaw > -179.5 else ''
        fps = 0

        time = str(datetime.timedelta(seconds=self.node.get_time()))[:10]

        if self.carla_status.fixed_delta_seconds:
            fps = 1 / self.carla_status.fixed_delta_seconds
        self._info_text = [
            'Frame: % 22s' % self.carla_status.frame,
            'Simulation time: % 12s' % time,
            'FPS: % 24.1f' % fps, '',
            'Vehicle: % 20s' % ' '.join(self.vehicle_info.type.split('.')[1:]),
            'Speed:   % 15.0f km/h' % (3.6 * self.vehicle_status.velocity),
            u'Heading:% 16.0f\N{DEGREE SIGN} % 2s' % (yaw, heading),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (x, y)),
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (self.latitude, self.longitude)),
            'Height:  % 18.0f m' % z, ''
        ]
        self._info_text += [
            ('Throttle:', self.vehicle_status.control.throttle, 0.0, 1.0),
            ('Steer:', self.vehicle_status.control.steer, -1.0, 1.0),
            ('Brake:', self.vehicle_status.control.brake, 0.0, 1.0),
            ('Reverse:', self.vehicle_status.control.reverse),
            ('Hand brake:', self.vehicle_status.control.hand_brake),
            ('Manual:', self.vehicle_status.control.manual_gear_shift),
            'Gear:        %s' % {
                -1: 'R',
                0: 'N'
            }.get(self.vehicle_status.control.gear, self.vehicle_status.control.gear), ''
        ]
        self._info_text += [('Manual ctrl:', self.manual_control)]
        if self.carla_status.synchronous_mode:
            self._info_text += [('Sync mode running:', self.carla_status.synchronous_mode_running)]
        if self.node.joystick_available:
            self._info_text += ['', '', 'Press <H> on Keyboard or', '<Share Button> on Xbox' ,'Controller for help']
        else:
            self._info_text += ['', '', 'Press <H> for help']

    def toggle_info(self):
        """
        show/hide the info text
        """
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        """
        display a notification for x seconds
        """
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        """
        display an error
        """
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        """
        render the display
        """
        if self._show_info:
            info_surface = pygame.Surface((250, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)
                                  ]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset + 50, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + int(f * (bar_width - 6)), v_offset + 8),
                                               (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8),
                                               (int(f * bar_width), 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    """
    Support Class for info display, fade out text
    """

    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        """
        set the text
        """
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, clock):
        """
        tick for fading
        """
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        """
        render the fading
        """
        display.blit(self.surface, self.pos)


# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """
    Show the help text
    """

    def __init__(self, font, width, height):
        lines = __doc__.split('\n')
        self.font = font
        self.dim = (680, height)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            line_height = round((self.dim[1] - 12) / len(lines))
            self.surface.blit(text_texture, (line_height, n * line_height))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        """
        Show/hide the help
        """
        self._render = not self._render

    def render(self, display):
        """
        render the help
        """
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================

def main(args=None):
    """
    main function
    """
    os.environ['SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS'] = "1"

    roscomp.init("manual_control", args=args)

    # resolution should be similar to spawned camera with role-name 'view'
    tmp_node = CompatibleNode("manual_control_param_helper")
    window_width = tmp_node.get_param("window_width", 800)
    window_height = tmp_node.get_param("window_height", 600)
    resolution = {"width": window_width, "height": window_height}
    tmp_node.destroy_node()

    pygame.init()
    pygame.font.init()
    pygame.display.set_caption("CARLA ROS manual control")
    pygame.joystick.init()

    try:
        num_joysticks = pygame.joystick.get_count()
        if num_joysticks > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            roscomp.loginfo("Enabled joystick: {}".format(joystick.get_name()))
        else:
            joystick = None
            roscomp.logwarn("No joystick found, using keyboard for control.")
            
        display = pygame.display.set_mode((resolution['width'], resolution['height']),
                                          pygame.HWSURFACE | pygame.DOUBLEBUF)

        manual_control_node = ManualControl(resolution, joystick)
        clock = pygame.time.Clock()

        executor = roscomp.executors.MultiThreadedExecutor()
        executor.add_node(manual_control_node)

        spin_thread = Thread(target=manual_control_node.spin)
        spin_thread.start()

        while roscomp.ok():
            clock.tick_busy_loop(60)
            if manual_control_node.render(clock, display):
                return
            pygame.display.flip()
    except KeyboardInterrupt:
        roscomp.loginfo("User requested shut down.")
    finally:
        roscomp.shutdown()
        spin_thread.join()
        pygame.quit()


if __name__ == '__main__':
    main()
