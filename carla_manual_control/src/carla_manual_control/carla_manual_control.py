#!/usr/bin/env python
#
# Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
OpenADSim Control

Automated driving is active while manual override is disabled.

If Xbox controller is connected, use the following trigger for control.

    right trigger (RT)          : throttle
    left trigger (LT)           : brake
    left joystick (left right)  : steer left/right
    left button (LB)            : toggle reverse
    right button (RB)           : hand-brake
    A button                    : toggle CARLA autopilot
    Y button                    : toggle manual override
    
    view button                 : toggle HUD
    Share button                : toggle help
    Xbox button                 : quit


If no Xbox controller is connected, use ARROWS or WASD keys for control.

    W            : throttle
    S            : brake
    AD           : steer
    Q            : toggle reverse
    Space        : hand-brake
    P            : toggle CARLA autopilot
    M            : toggle manual transmission
    ,/.          : gear up/down
    B            : toggle manual override

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


def load_openads_logo(size=48):
    """Load the packaged OpenADS mark at the requested display size."""
    path = os.path.join(os.path.dirname(__file__), 'assets', 'openads-mark.png')
    try:
        logo = pygame.image.load(path).convert_alpha()
        return pygame.transform.smoothscale(logo, (size, size))
    except (pygame.error, OSError):
        return None


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

        # Disable autopilot if controller input is detected
        if input and self.autopilot_enabled:
            self.autopilot_enabled = False
            self.set_autopilot(False)
            self.hud.notification('CARLA autopilot off')

        # Activate vehicle_control_manual_override if input detected.
        if input and not self.vehicle_control_manual_override:
            self.vehicle_control_manual_override = True
            self.set_vehicle_control_manual_override(True)

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
                    self.hud.notification('CARLA autopilot %s' %
                                          ('on' if self.node.autopilot_enabled else 'off'))
        
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
                    self.hud.notification('CARLA autopilot %s' %
                                         ('on' if self.node.autopilot_enabled else 'off'))
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
        notification_font = pygame.font.Font(pygame.font.get_default_font(), 20)
        self._font_title = self._make_font('inter,dejavusans,ubuntusans,arial', 18, True)
        self._font_status = self._make_font('inter,dejavusans,ubuntusans,arial', 20, True)
        self._font_speed = self._make_font('inter,dejavusans,ubuntusans,arial', 46, True)
        self._font_speed_compact = self._make_font('inter,dejavusans,ubuntusans,arial', 40, True)
        self._font_body = self._make_font('inter,dejavusans,ubuntusans,arial', 13)
        self._font_label = self._make_font('inter,dejavusans,ubuntusans,arial', 11, True)
        self._font_mono = self._make_font('ubuntumono,dejavusansmono,monospace', 13)
        self._notifications = FadingText(notification_font, (width, 40), (0, height - 40))
        self.help = HelpText(width, height)
        self._logo = load_openads_logo()
        self._show_info = True
        self.vehicle_status = CarlaEgoVehicleStatus()
        self._display_steer = 0.0

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
        self.manual_control_received = False

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
        target_steer = self.vehicle_status.control.steer
        blend = min(1.0, 18.0 * 1e-3 * clock.get_time())
        self._display_steer += (target_steer - self._display_steer) * blend

    def carla_status_updated(self, data):
        """
        Callback on carla status
        """
        self.carla_status = data

    def manual_control_override_updated(self, data):
        """
        Callback on vehicle status updates
        """
        self.manual_control = data.data
        self.manual_control_received = True

    def vehicle_status_updated(self, vehicle_status):
        """
        Callback on vehicle status updates
        """
        self.vehicle_status = vehicle_status

    def vehicle_info_updated(self, vehicle_info):
        """
        Callback on vehicle info updates
        """
        self.vehicle_info = vehicle_info

    def gnss_updated(self, data):
        """
        Callback on gnss position updates
        """
        self.latitude = data.latitude
        self.longitude = data.longitude

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
            self._render_panel(display)
        self._notifications.render(display)
        self.help.render(display)

    @staticmethod
    def _make_font(names, size, bold=False):
        font_path = pygame.font.match_font(names)
        font = pygame.font.Font(font_path, size) if font_path else pygame.font.Font(None, size)
        font.set_bold(bold)
        return font

    @staticmethod
    def _card(surface, rect, color=(14, 24, 32, 230), border=(55, 74, 84, 230)):
        pygame.draw.rect(surface, color, rect, border_radius=11)
        pygame.draw.rect(surface, border, rect, width=1, border_radius=11)

    @staticmethod
    def _clamp(value, lower=0.0, upper=1.0):
        return max(lower, min(upper, value))

    def _render_logo(self, surface, rect):
        if self._logo:
            logo_rect = self._logo.get_rect(center=rect.center)
            surface.blit(self._logo, logo_rect)
            return

        # A compact fallback keeps the header branded even when an installed
        # package is missing its optional image asset.
        line_width = max(3, rect.width // 9)
        pygame.draw.arc(surface, (105, 221, 203), rect, 0.45, 3.85, line_width)
        pygame.draw.arc(surface, (35, 154, 173), rect, 3.9, 6.4, line_width)
        pygame.draw.line(surface, (57, 194, 196), rect.midbottom,
                         (rect.centerx + 8, rect.top + 10), line_width)

    def _text(self, surface, text, font, color, pos):
        rendered = font.render(str(text), True, color)
        surface.blit(rendered, pos)
        return rendered.get_rect(topleft=pos)

    def _right_text(self, surface, text, font, color, right, y):
        rendered = font.render(str(text), True, color)
        surface.blit(rendered, (right - rendered.get_width(), y))

    @staticmethod
    def _fit_text(text, font, max_width):
        text = str(text)
        if font.size(text)[0] <= max_width:
            return text
        while len(text) > 4 and font.size(text + '...')[0] > max_width:
            text = text[:-1]
        return text + '...'

    def _render_panel(self, display):
        panel_margin = 12 if self.dim[1] >= 560 else 8
        panel_width = min(320, max(280, int(self.dim[0] * 0.4)))
        available_height = self.dim[1] - 2 * panel_margin
        compact = available_height < 560
        padding = 10
        gap = 5 if compact else 6
        content_width = panel_width - 2 * padding
        heights = {
            'header': 50,
            'mode': 58 if compact else 62,
            'speed': 72 if compact else 82,
            'input': 92 if compact else 104,
            'position': 92 if compact else 104,
            'simulation': 68 if compact else 82,
        }
        content_height = sum(heights.values()) + gap * (len(heights) - 1)
        footer_height = 17
        panel_height = min(
            available_height,
            2 * padding + content_height + gap + footer_height)
        panel = pygame.Surface((panel_width, panel_height), pygame.SRCALPHA)
        pygame.draw.rect(panel, (7, 14, 20, 210), panel.get_rect(), border_radius=16)
        pygame.draw.rect(panel, (61, 80, 89, 220), panel.get_rect(), width=1, border_radius=16)

        y = padding
        self._render_header(panel, pygame.Rect(padding, y, content_width, heights['header']))
        y += heights['header'] + gap
        self._render_mode(panel, pygame.Rect(padding, y, content_width, heights['mode']))
        y += heights['mode'] + gap
        self._render_speed(panel, pygame.Rect(padding, y, content_width, heights['speed']), compact)
        y += heights['speed'] + gap
        self._render_vehicle_input(panel, pygame.Rect(padding, y, content_width, heights['input']), compact)
        y += heights['input'] + gap
        self._render_position(panel, pygame.Rect(padding, y, content_width, heights['position']), compact)
        y += heights['position'] + gap
        self._render_simulation(panel, pygame.Rect(padding, y, content_width, heights['simulation']), compact)

        footer_y = y + heights['simulation'] + gap
        help_hint = 'H / SHARE  Help' if self.node.joystick_available else 'H  Help'
        self._text(panel, help_hint, self._font_label, (139, 157, 168),
                   (padding + 2, footer_y))
        self._right_text(panel, 'F1  Hide HUD', self._font_label,
                         (139, 157, 168), panel_width - padding - 2, footer_y)
        display.blit(panel, (panel_margin, panel_margin))

    def _render_header(self, surface, rect):
        logo_rect = pygame.Rect(rect.x, rect.y + 1, 48, 48)
        self._render_logo(surface, logo_rect)
        self._text(surface, 'OpenADSim', self._font_title, (238, 246, 249),
                   (rect.x + 56, rect.y + 5))
        self._text(surface, 'CONTROL', self._font_label, (79, 203, 196),
                   (rect.x + 57, rect.y + 29))

    def _render_mode(self, surface, rect):
        if not self.manual_control_received:
            color = (80, 94, 104)
            background = (31, 42, 49, 230)
            state = 'WAITING FOR STATUS'
        elif self.manual_control:
            color = (255, 179, 71)
            background = (61, 43, 20, 230)
            state = 'INACTIVE'
        else:
            color = (68, 222, 164)
            background = (18, 59, 51, 230)
            state = 'ACTIVE'

        border = tuple(max(40, int(component * 0.55)) for component in color) + (225,)
        self._card(surface, rect, background, border)
        pygame.draw.circle(surface, color, (rect.x + 17, rect.y + 17), 5)
        self._text(surface, 'AUTOMATED DRIVING', self._font_label, color,
                   (rect.x + 29, rect.y + 9))
        self._text(surface, state, self._font_status, (244, 250, 251),
                   (rect.x + 14, rect.y + 29))
        self._right_text(surface, 'B / Y', self._font_label, (164, 181, 188),
                         rect.right - 12, rect.y + 10)

    def _render_speed(self, surface, rect, compact):
        self._card(surface, rect)
        speed = max(0, int(round(3.6 * self.vehicle_status.velocity)))
        speed_font = self._font_speed_compact if compact else self._font_speed
        speed_texture = speed_font.render(str(speed), True, (245, 250, 251))
        speed_y = rect.y + (20 if compact else 23)
        surface.blit(speed_texture, (rect.x + 13, speed_y))
        self._text(surface, 'SPEED', self._font_label, (124, 147, 159),
                   (rect.x + 14, rect.y + 8))
        self._text(surface, 'km/h', self._font_label, (124, 147, 159),
                   (rect.x + 18 + speed_texture.get_width(), speed_y + speed_texture.get_height() - 17))

        control = self.vehicle_status.control
        gear = {-1: 'R', 0: 'N'}.get(control.gear, str(control.gear))
        heading = self._heading_text(self.yaw)
        divider_x = rect.right - 96
        pygame.draw.line(surface, (55, 74, 84),
                         (divider_x, rect.y + 12), (divider_x, rect.bottom - 12))
        self._text(surface, 'GEAR', self._font_label, (124, 147, 159),
                   (divider_x + 13, rect.y + 9))
        self._text(surface, gear, self._font_status, (245, 250, 251),
                   (divider_x + 13, rect.y + 26))
        self._right_text(surface, heading, self._font_mono, (177, 198, 207),
                         rect.right - 12, rect.bottom - 23)

    @staticmethod
    def _heading_text(yaw):
        # ROS yaw is counter-clockwise from +x (east). A compass heading is
        # clockwise from north, hence the 90-degree offset and sign change.
        heading = (90.0 - yaw) % 360.0
        cardinal_directions = ('N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW')
        cardinal = cardinal_directions[int((heading + 22.5) // 45.0) % 8]
        rounded_heading = int(round(heading)) % 360
        return u'{:03d}\N{DEGREE SIGN} {}'.format(rounded_heading, cardinal)

    def _render_vehicle_input(self, surface, rect, compact):
        self._card(surface, rect)
        self._text(surface, 'VEHICLE INPUT', self._font_label, (124, 147, 159),
                   (rect.x + 12, rect.y + 8))
        control = self.vehicle_status.control
        first_y = rect.y + 28
        row_gap = 17 if compact else 18
        self._render_input_bar(surface, rect, first_y, 'Throttle', control.throttle,
                               (79, 203, 196))
        self._render_input_bar(surface, rect, first_y + row_gap, 'Brake', control.brake,
                               (255, 179, 71))
        self._render_steering(surface, rect, first_y + 2 * row_gap, self._display_steer)

        chip_y = rect.bottom - 21
        chip_gap = 5
        chip_width = (rect.width - 24 - 2 * chip_gap) // 3
        self._render_chip(surface, pygame.Rect(rect.x + 12, chip_y, chip_width, 14),
                          'REVERSE', control.reverse)
        self._render_chip(surface, pygame.Rect(rect.x + 12 + chip_width + chip_gap,
                                               chip_y, chip_width, 14),
                          'HAND BRAKE', control.hand_brake)
        self._render_chip(surface, pygame.Rect(rect.x + 12 + 2 * (chip_width + chip_gap),
                                               chip_y, chip_width, 14),
                          'MANUAL GEAR', control.manual_gear_shift)

    def _render_input_bar(self, surface, card_rect, y, label, value, color):
        self._text(surface, label, self._font_label, (169, 187, 196),
                   (card_rect.x + 12, y - 3))
        bar = pygame.Rect(card_rect.x + 82, y, card_rect.width - 96, 7)
        pygame.draw.rect(surface, (54, 72, 81), bar, border_radius=4)
        fill_width = int(bar.width * self._clamp(value))
        if fill_width:
            pygame.draw.rect(surface, color, (bar.x, bar.y, fill_width, bar.height), border_radius=4)

    def _render_steering(self, surface, card_rect, y, value):
        self._text(surface, 'Steering', self._font_label, (169, 187, 196),
                   (card_rect.x + 12, y - 3))
        bar = pygame.Rect(card_rect.x + 82, y, card_rect.width - 96, 7)
        pygame.draw.rect(surface, (54, 72, 81), bar, border_radius=4)
        end = bar.x + int(self._clamp((value + 1.0) / 2.0) * bar.width)
        if end != bar.centerx:
            fill = pygame.Rect(min(end, bar.centerx), bar.y,
                               max(2, abs(end - bar.centerx)), bar.height)
            pygame.draw.rect(surface, (79, 203, 196), fill, border_radius=4)
        pygame.draw.line(surface, (168, 188, 196),
                         (bar.centerx, bar.y - 2), (bar.centerx, bar.bottom + 2), 2)

    def _render_chip(self, surface, rect, label, active):
        background = (65, 91, 95) if active else (44, 58, 65)
        foreground = (137, 232, 211) if active else (151, 168, 176)
        pygame.draw.rect(surface, background, rect, border_radius=7)
        texture = self._font_label.render(label, True, foreground)
        surface.blit(texture, (rect.centerx - texture.get_width() // 2,
                               rect.centery - texture.get_height() // 2))

    def _render_position(self, surface, rect, compact):
        self._card(surface, rect)
        self._text(surface, 'VEHICLE & POSITION', self._font_label, (124, 147, 159),
                   (rect.x + 12, rect.y + 8))
        vehicle = ' '.join(self.vehicle_info.type.split('.')[1:]) or 'unknown vehicle'
        if vehicle.split()[-1] == 'karl':
            vehicle += '.'
        vehicle = self._fit_text(vehicle, self._font_body, rect.width - 105)
        row_y = rect.y + 27
        row_gap = 16 if compact else 18
        self._render_value_row(surface, rect, row_y, 'Vehicle', vehicle)
        self._render_value_row(surface, rect, row_y + row_gap, 'Location',
                               '({:.1f}, {:.1f})'.format(self.x, self.y))
        self._render_value_row(surface, rect, row_y + 2 * row_gap, 'GNSS',
                               '{:.5f}, {:.5f}'.format(self.latitude, self.longitude))
        self._render_value_row(surface, rect, row_y + 3 * row_gap, 'Height',
                               '{:.0f} m'.format(self.z))

    def _render_value_row(self, surface, rect, y, label, value):
        self._text(surface, label, self._font_label, (132, 153, 164), (rect.x + 12, y))
        self._right_text(surface, value, self._font_mono, (219, 230, 234),
                         rect.right - 12, y - 1)

    def _render_simulation(self, surface, rect, compact):
        self._card(surface, rect)
        self._text(surface, 'SIMULATION', self._font_label, (124, 147, 159),
                   (rect.x + 12, rect.y + 8))
        fps = 0.0
        if self.carla_status.fixed_delta_seconds:
            fps = 1.0 / self.carla_status.fixed_delta_seconds
        simulation_time = str(datetime.timedelta(seconds=self.node.get_time()))[:10]
        sync_state = 'OFF'
        if self.carla_status.synchronous_mode:
            sync_state = 'RUNNING' if self.carla_status.synchronous_mode_running else 'PAUSED'

        first_y = rect.y + 29
        second_y = rect.y + (47 if compact else 51)
        self._text(surface, 'TIME', self._font_label, (132, 153, 164), (rect.x + 12, first_y))
        self._text(surface, simulation_time, self._font_mono, (219, 230, 234),
                   (rect.x + 50, first_y - 1))
        self._right_text(surface, '{:.1f} FPS'.format(fps), self._font_mono,
                         (219, 230, 234), rect.right - 12, first_y - 1)
        self._text(surface, 'FRAME', self._font_label, (132, 153, 164), (rect.x + 12, second_y))
        self._text(surface, str(self.carla_status.frame), self._font_mono, (219, 230, 234),
                   (rect.x + 56, second_y - 1))
        self._right_text(surface, 'SYNC ' + sync_state, self._font_label,
                         (79, 203, 196) if sync_state == 'RUNNING' else (132, 153, 164),
                         rect.right - 12, second_y)


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
        self.surface = pygame.Surface(self.dim, pygame.SRCALPHA)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        """
        set the text
        """
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim, pygame.SRCALPHA)
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

    def __init__(self, width, height):
        self.screen_dim = (width, height)
        self.dim = (min(720, width - 32), min(500, height - 32))
        self.pos = ((width - self.dim[0]) // 2, (height - self.dim[1]) // 2)
        self._font_title = HUD._make_font('inter,dejavusans,ubuntusans,arial', 22, True)
        self._font_heading = HUD._make_font('inter,dejavusans,ubuntusans,arial', 11, True)
        self._font_key = HUD._make_font('ubuntumono,dejavusansmono,monospace', 11, True)
        self._font_body = HUD._make_font('inter,dejavusans,ubuntusans,arial', 12)
        self._logo = load_openads_logo()
        self.surface = pygame.Surface(self.dim, pygame.SRCALPHA)
        self._build_surface()
        self._render = False

    def _build_surface(self):
        pygame.draw.rect(self.surface, (7, 14, 20, 235), self.surface.get_rect(), border_radius=18)
        pygame.draw.rect(self.surface, (61, 80, 89, 225), self.surface.get_rect(),
                         width=1, border_radius=18)

        if self._logo:
            self.surface.blit(self._logo, (19, 14))
        else:
            pygame.draw.circle(self.surface, (79, 203, 196), (43, 38), 17, width=4)
        self.surface.blit(self._font_title.render('OpenADSim Control', True, (240, 247, 249)),
                          (76, 16))
        self.surface.blit(self._font_body.render('Keyboard and controller reference', True,
                                                 (133, 155, 166)), (77, 43))
        self._pill(self.surface, 'H / ?', self.dim[0] - 76, 25, 52)
        pygame.draw.line(self.surface, (48, 65, 74),
                         (20, 70), (self.dim[0] - 20, 70))

        column_gap = 28
        column_width = (self.dim[0] - 48 - column_gap) // 2
        left_x = 24
        right_x = left_x + column_width + column_gap
        pygame.draw.line(self.surface, (44, 60, 68),
                         (self.dim[0] // 2, 86), (self.dim[0] // 2, self.dim[1] - 48))

        keyboard = [
            ('DRIVING', [
                ('W / UP', 'Accelerate'), ('S / DOWN', 'Brake'),
                ('A D / ARROWS', 'Steer left / right'), ('Q', 'Toggle reverse'),
                ('SPACE', 'Hand brake')]),
            ('AUTOMATION', [
                ('B', 'Toggle manual override'), ('P', 'Toggle CARLA autopilot')]),
            ('TRANSMISSION', [
                ('M', 'Automatic / manual'), (', / .', 'Gear down / up')]),
            ('INTERFACE', [
                ('F1', 'Toggle HUD'), ('H / ?', 'Toggle help'), ('ESC', 'Quit')]),
        ]
        controller = [
            ('DRIVING', [
                ('RT', 'Accelerate'), ('LT', 'Brake'), ('LEFT STICK', 'Steer'),
                ('LB', 'Toggle reverse'), ('RB', 'Hand brake')]),
            ('AUTOMATION', [
                ('Y', 'Toggle manual override'), ('A', 'Toggle CARLA autopilot')]),
            ('INTERFACE', [
                ('VIEW', 'Toggle HUD'), ('SHARE', 'Toggle help'), ('XBOX', 'Quit')]),
        ]

        self.surface.blit(self._font_heading.render('KEYBOARD', True, (79, 203, 196)),
                          (left_x, 84))
        self.surface.blit(self._font_heading.render('XBOX CONTROLLER', True, (79, 203, 196)),
                          (right_x, 84))
        self._render_sections(keyboard, left_x, 108, column_width)
        self._render_sections(controller, right_x, 108, column_width)

    def _render_sections(self, sections, x, y, width):
        for heading, rows in sections:
            heading_texture = self._font_heading.render(heading, True, (121, 143, 154))
            self.surface.blit(heading_texture, (x, y))
            y += 18
            for key, description in rows:
                self._pill(self.surface, key, x, y, 92)
                description = HUD._fit_text(description, self._font_body, width - 106)
                text = self._font_body.render(description, True, (222, 232, 236))
                self.surface.blit(text, (x + 104, y + 2))
                y += 22
            y += 8

    def _pill(self, surface, text, x, y, width):
        rect = pygame.Rect(x, y, width, 17)
        pygame.draw.rect(surface, (43, 57, 64), rect, border_radius=5)
        pygame.draw.rect(surface, (66, 84, 92), rect, width=1, border_radius=5)
        texture = self._font_key.render(text, True, (173, 217, 216))
        surface.blit(texture, (rect.centerx - texture.get_width() // 2,
                               rect.centery - texture.get_height() // 2))

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
            backdrop = pygame.Surface(self.screen_dim, pygame.SRCALPHA)
            backdrop.fill((0, 0, 0, 145))
            display.blit(backdrop, (0, 0))
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
    pygame.display.set_caption("OpenADSim Control")
    icon = load_openads_logo(64)
    if icon:
        pygame.display.set_icon(icon)
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
