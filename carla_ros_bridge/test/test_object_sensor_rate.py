"""object pseudo-sensor sampling tests; no CARLA server or ROS node required."""

import math
import unittest
from unittest.mock import Mock

from carla_ros_bridge.object_sensor import ObjectSensor
from carla_ros_bridge.vehicle import Vehicle
from derived_object_msgs.msg import Object


class ObjectSensorRateTest(unittest.TestCase):
    def make_sensor(self, rate=0.0, parent=None):
        node = Mock()
        node.parameters = {
            'object_sensor_publish_rate_hz': rate,
            'publish_static_vehicles': True,
            'start_unix_time_stamp': 0.0,
        }
        node.get_time.return_value = 0.0
        world = Mock()
        world.get_environment_objects.return_value = []
        vehicle = Mock(spec=Vehicle)
        vehicle.get_object_info.return_value = Object(id=42)
        sensor = ObjectSensor(1, 'objects', parent, node, {42: vehicle}, world)
        sensor.get_msg_header = Mock(wraps=sensor.get_msg_header)
        return sensor

    def publish_frames(self, sensor, timestamps):
        for frame, timestamp in enumerate(timestamps):
            sensor.update(frame, timestamp)
        return [call.kwargs['timestamp'] for call in sensor.get_msg_header.call_args_list]

    def test_default_publishes_every_frame(self):
        sensor = self.make_sensor()
        stamps = [1.0 + frame * 0.01 for frame in range(100)]
        self.assertEqual(self.publish_frames(sensor, stamps), stamps)
        self.assertEqual(sensor.object_publisher.publish.call_count, len(stamps))

    def test_100hz_to_20hz_preserves_timestamps_and_objects(self):
        ego = Mock(uid=7)
        ego.get_prefix.return_value = 'ego_vehicle'
        for parent in (None, ego):
            with self.subTest(parent=parent):
                sensor = self.make_sensor(20.0, parent)
                # CARLA's single-precision timestep is slightly shorter than 10 ms
                stamps = [1.0 + frame * 0.009999999776482582 for frame in range(1000)]
                self.assertEqual(self.publish_frames(sensor, stamps), stamps[::5])
                messages = sensor.object_publisher.publish.call_args_list
                self.assertEqual(len(messages), 200)
                self.assertTrue(all(call.args[0].objects[0].id == 42 for call in messages))

    def test_50hz_to_20hz_retains_phase(self):
        sensor = self.make_sensor(20.0)
        stamps = [1.0 + frame * 0.019999999552965164 for frame in range(500)]
        published = self.publish_frames(sensor, stamps)
        self.assertEqual(len(published), 200)
        gaps = {round((b - a) * 1000) for a, b in zip(published, published[1:])}
        self.assertEqual(gaps, {40, 60})

    def test_skipped_ticks_do_not_build_objects_or_query_static_vehicles(self):
        sensor = self.make_sensor(20.0)
        sensor.update(1, 1.0)
        queries = sensor.world.get_environment_objects.call_count
        objects = sensor.actor_list[42].get_object_info.call_count
        for frame in range(1, 5):
            sensor.update(frame + 1, 1.0 + frame * 0.01)
        self.assertEqual(sensor.world.get_environment_objects.call_count, queries)
        self.assertEqual(sensor.actor_list[42].get_object_info.call_count, objects)
        self.assertEqual(sensor.object_publisher.publish.call_count, 1)

    def test_time_jumps_reset_or_skip_deadlines_without_bursts(self):
        sensor = self.make_sensor(20.0)
        stamps = [1.0, 1.01, 10.0, 10.01, 0.5, 0.51, 0.55]
        self.assertEqual(self.publish_frames(sensor, stamps), [1.0, 10.0, 0.5, 0.55])

    def test_requested_rate_above_tick_rate_publishes_each_tick(self):
        sensor = self.make_sensor(100.0)
        stamps = [1.0 + frame * 0.02 for frame in range(100)]
        self.assertEqual(self.publish_frames(sensor, stamps), stamps)

    def test_invalid_rates_are_rejected(self):
        for rate in (-1.0, math.inf, math.nan):
            with self.subTest(rate=rate), self.assertRaises(ValueError):
                self.make_sensor(rate)


if __name__ == '__main__':
    unittest.main()
