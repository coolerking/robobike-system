"""Bridge node against a fake ROBOBIKE; runs with real rclpy or with stubs."""

import math
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import ros_stubs  # noqa: E402

REAL_ROS = ros_stubs.install()

import rclpy  # noqa: E402
from fake_robobike import FakeRobobike  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from robobike_bridge import bridge_node  # noqa: E402
from robobike_bridge.bridge_node import BridgeNode  # noqa: E402
from robobike_bridge.drive import BT_F, BT_S, BT_STR_S  # noqa: E402

LINE13 = "a,{t},{drv},1.5,36.0,0.0,4.0,-0.1,-0.2,-0.3,2.2,9.3,2.1"


def setUpModule():
    if REAL_ROS:
        rclpy.init()


def tearDownModule():
    if REAL_ROS:
        rclpy.shutdown()


def published(publisher):
    return [call.args[0] for call in publisher.publish.call_args_list]


class BridgeTestCase(unittest.TestCase):
    def setUp(self):
        self.robot = FakeRobobike()
        self.addCleanup(self.robot.close)

    def make_node(self, **params):
        params.setdefault("base_url", self.robot.base_url)
        params.setdefault("http_timeout", 0.5)
        node = BridgeNode(parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        self.addCleanup(node.destroy_node)
        for name in ("telemetry_publisher", "state_publisher", "connected_publisher", "diagnostics_publisher"):
            setattr(node, name, MagicMock(name=name))
        return node

    def cmd_vel(self, node, linear, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        node.on_cmd_vel(msg)


class ParameterTest(BridgeTestCase):
    def test_invalid_parameters_raise(self):
        for params in (
            {"poll_period": 0.0}, {"poll_period": 5.0}, {"poll_period": math.nan},
            {"http_timeout": math.inf}, {"http_timeout": -1.0}, {"qos_depth": 0},
            {"stale_timeout": 0.0}, {"reconnect_backoff_max": 0.0}, {"cmd_timeout": 0.0},
            {"control_period": math.nan}, {"start_linear": 0.02, "stop_linear": 0.02},
            {"min_drive_speed": 61}, {"base_url": "ftp://192.168.4.1"},
        ):
            with self.subTest(params=params), self.assertRaises(ValueError):
                self.make_node(**params)

    def test_defaults(self):
        node = BridgeNode()
        self.addCleanup(node.destroy_node)
        self.assertEqual(node.base_url, "http://192.168.4.1")
        self.assertEqual(node.poll_period, 0.05)
        self.assertEqual(node.telemetry_topic, "/robobike/telemetry")
        self.assertFalse(node.enable_drive)
        self.assertIsNone(node.cmd_subscription)
        self.assertEqual(node.drive_state, "DISABLED")


class TelemetryTest(BridgeTestCase):
    def test_publishes_every_sample_in_order(self):
        node = self.make_node(frame_id="bike")
        self.robot.queue_rows("a,1,0,0,0,0,0,0")  # stale sample discarded by /clear_buffer
        self.assertEqual(node.poll_telemetry_once(), 0)
        self.robot.queue_rows(LINE13.format(t=1000, drv=0), "junk", LINE13.format(t=1004, drv=0))
        self.assertEqual(node.poll_telemetry_once(), 2)
        msgs = published(node.telemetry_publisher)
        self.assertEqual([m.time_ms for m in msgs], [1000, 1004])
        self.assertEqual(msgs[0].header.frame_id, "bike")
        self.assertEqual(msgs[0].record_type, "a")
        self.assertAlmostEqual(msgs[0].sv_std, 36.0)
        self.assertAlmostEqual(msgs[0].acc_y, 9.3, places=5)
        stamp_ns = [m.header.stamp.sec * 10**9 + m.header.stamp.nanosec for m in msgs]
        self.assertEqual(stamp_ns[1] - stamp_ns[0], 4_000_000)
        self.assertEqual(self.robot.paths, ["/clear_buffer", "/get_acc", "/get_acc"])
        self.assertEqual(node.malformed, 1)
        self.assertEqual(published(node.connected_publisher)[-1].data, True)

    def test_8_column_firmware_gives_nan_extensions(self):
        node = self.make_node(clear_buffer_on_start=False)
        self.robot.queue_rows("a,175715,47.000,24.435,80.000,-1.340,27.215,0.095")
        node.poll_telemetry_once()
        msg = published(node.telemetry_publisher)[0]
        self.assertTrue(math.isnan(msg.acc_x))
        self.assertEqual(self.robot.paths, ["/get_acc"])

    def test_gap_and_rewind_are_counted(self):
        node = self.make_node()
        node.poll_telemetry_once()
        self.robot.queue_rows(*(LINE13.format(t=t, drv=0) for t in (1000, 1004, 1020)))
        node.poll_telemetry_once()
        self.assertEqual(node.gaps.missing, 3)
        self.robot.queue_rows(LINE13.format(t=8, drv=0))
        node.poll_telemetry_once()
        self.assertEqual(node.clock.rewinds, 1)

    def test_errors_back_off_then_reconnect_with_clear(self):
        node = self.make_node()
        node.poll_telemetry_once()
        self.robot.fail_next = 500
        self.assertIsNone(node.poll_telemetry_once())
        self.assertEqual((node.http_errors, node.backoff), (1, 0.5))
        self.robot.close()
        node.poll_telemetry_once()
        self.assertEqual((node.http_errors, node.backoff), (2, 1.0))

    def test_reconnect_clears_buffer_again(self):
        node = self.make_node()
        node.poll_telemetry_once()
        self.robot.fail_next = 500
        node.poll_telemetry_once()
        self.robot.queue_rows(LINE13.format(t=2000, drv=0))
        node.poll_telemetry_once()
        self.assertEqual(self.robot.paths, ["/clear_buffer", "/get_acc", "/get_acc", "/clear_buffer", "/get_acc"])
        self.assertEqual(node.backoff, 0.0)

    def test_stale_telemetry_reports_disconnected(self):
        node = self.make_node(stale_timeout=0.05)
        node.poll_telemetry_once()
        self.robot.queue_rows(LINE13.format(t=1000, drv=0))
        node.poll_telemetry_once()
        self.assertTrue(node.connected())
        time.sleep(0.08)
        node.control_tick()
        self.assertFalse(node.connected())
        self.assertEqual(published(node.connected_publisher)[-1].data, False)

    def test_diagnostics_has_telemetry_and_drive_status(self):
        node = self.make_node()
        node.publish_diagnostics()
        statuses = published(node.diagnostics_publisher)[0].status
        self.assertEqual([s.name for s in statuses], ["robobike_bridge: telemetry", "robobike_bridge: drive"])

    def test_worker_threads_poll_in_background(self):
        node = self.make_node(poll_period=0.01)
        self.robot.auto_rows = True
        node.start_workers()
        deadline = time.monotonic() + 2.0
        while not node.telemetry_publisher.publish.called and time.monotonic() < deadline:
            time.sleep(0.01)
        node.stop_workers()
        self.assertTrue(node.telemetry_publisher.publish.called)


class ReadOnlyTest(BridgeTestCase):
    def test_enable_drive_false_never_sends_commands(self):
        node = self.make_node()
        self.robot.auto_rows = True
        for _ in range(5):
            node.poll_telemetry_once()
            node.control_tick()
            node.send_pending_commands()
        node.shutdown_drive()
        self.assertEqual(self.robot.commands, [])
        self.assertNotIn("/", self.robot.paths)
        self.assertNotIn("/command", self.robot.paths)


class DriveTest(BridgeTestCase):
    def make_drive_node(self, **params):
        node = self.make_node(enable_drive=True, **params)
        self.robot.auto_rows = True
        return node

    def cycle(self, node, linear=None, angular=0.0):
        if linear is not None:
            self.cmd_vel(node, linear, angular)
        node.poll_telemetry_once()
        node.control_tick()
        node.send_pending_commands()

    def test_subscribes_to_cmd_vel(self):
        node = self.make_drive_node(cmd_topic="/bike/cmd")
        self.assertEqual(node.cmd_subscription.topic if not REAL_ROS else node.cmd_subscription.topic_name, "/bike/cmd")
        self.assertEqual(node.drive_state, "STOPPED")

    def test_start_steer_stop_round_trip(self):
        node = self.make_drive_node(steer_min_interval=0.0)
        self.cycle(node, 0.2)
        self.assertEqual(self.robot.commands, [(BT_F, None)])
        self.cycle(node, 0.2)
        self.assertEqual(node.drive_state, "RUNNING")
        self.cycle(node, 0.2, 0.5)
        self.assertEqual(self.robot.commands[-1], (BT_STR_S, "-50"))
        self.assertEqual(node.mot_spd, 60)
        self.cycle(node, 0.0)
        self.assertEqual(self.robot.commands[-1], (BT_S, None))
        self.cycle(node, 0.0)
        self.assertEqual(node.drive_state, "STOPPED")
        self.assertEqual(
            [m.data for m in published(node.state_publisher)], ["STARTING", "RUNNING", "STOPPING", "STOPPED"]
        )
        self.assertNotIn("/", self.robot.paths)

    def test_shutdown_stops_running_bike(self):
        node = self.make_drive_node()
        self.cycle(node, 0.2)
        self.cycle(node, 0.2)
        self.assertEqual(node.drive_state, "RUNNING")
        node.shutdown_drive()
        self.assertEqual(self.robot.commands[-1], (BT_S, None))
        self.assertEqual(self.robot.sv_drv, 0.0)
        self.assertEqual(node.drive_state, "STOPPED")

    def test_lost_telemetry_stops(self):
        node = self.make_drive_node(stale_timeout=0.05)
        self.cycle(node, 0.2)
        self.cycle(node, 0.2)
        time.sleep(0.08)
        self.cmd_vel(node, 0.2)
        node.control_tick()
        node.send_pending_commands()
        self.assertEqual(self.robot.commands[-1], (BT_S, None))

    def test_command_errors_are_counted(self):
        node = self.make_drive_node()
        self.cycle(node, 0.2)
        self.robot.close()
        node.command_queue.put(bridge_node.Command(BT_S))
        node.send_pending_commands()
        self.assertEqual(node.command_errors, 1)


if __name__ == "__main__":
    unittest.main()
