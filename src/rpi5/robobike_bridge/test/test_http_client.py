"""Checks for the keep-alive HTTP client against a fake ROBOBIKE server (no ROS needed)."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from fake_robobike import FakeRobobike  # noqa: E402
from robobike_bridge.http_client import RobobikeHttpClient, RobobikeHttpError  # noqa: E402


class HttpClientTest(unittest.TestCase):
    def setUp(self):
        self.robot = FakeRobobike()
        self.addCleanup(self.robot.close)
        self.client = RobobikeHttpClient(self.robot.base_url, timeout=0.5)
        self.addCleanup(self.client.close)

    def test_get_acc_and_clear_buffer(self):
        self.robot.queue_rows("a,1,0,0,0,0,0,0", "a,5,0,0,0,0,0,0")
        self.assertEqual(self.client.get_acc(), "a,1,0,0,0,0,0,0\na,5,0,0,0,0,0,0\n")
        self.assertEqual(self.client.get_acc(), "")
        self.client.clear_buffer()
        self.assertEqual(self.robot.paths, ["/get_acc", "/get_acc", "/clear_buffer"])

    def test_connection_is_reused(self):
        for _ in range(3):
            self.client.get_acc()
        self.assertEqual(self.robot.connections, 1)

    def test_command_query_and_settings(self):
        settings = self.client.command(7, -40)
        self.assertEqual(self.robot.commands, [(7, "-40")])
        self.assertEqual(settings["MOT_SPD"], 60)

    def test_forbidden_buttons_are_rejected_locally(self):
        for button in (0, 5, 6, 11, 12, 13, 36):
            with self.subTest(button=button), self.assertRaises(ValueError):
                self.client.command(button)
        self.assertEqual(self.robot.paths, [])

    def test_errors_raise_and_reconnect(self):
        self.robot.fail_next = 500
        with self.assertRaises(RobobikeHttpError):
            self.client.get_acc()
        self.assertEqual(self.client.get_acc(), "")

    def test_unreachable_host_raises(self):
        self.robot.close()
        with self.assertRaises(RobobikeHttpError):
            self.client.get_acc()


if __name__ == "__main__":
    unittest.main()
