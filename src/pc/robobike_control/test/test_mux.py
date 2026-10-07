"""Regression checks for planar command arbitration, without a ROS installation."""

import unittest
from unittest.mock import patch

from robobike_control.mux import CommandMux


class CommandMuxTest(unittest.TestCase):
    def setUp(self):
        self.clock_patch = patch("robobike_control.mux.time.monotonic", return_value=10.0)
        self.clock = self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        self.mux = CommandMux()

    def test_standby_ignores_both_sources(self):
        self.mux.receive("AUTO", 0.3, 0.5)
        self.mux.receive("TELEOP", 0.3, 0.5)
        self.assertEqual(self.mux.output(), (0.0, 0.0))

    def test_auto_clamps_velocity_and_ignores_idle_teleop(self):
        self.mux.set_mode("AUTO")
        self.mux.receive("AUTO", 9.0, -9.0)
        self.mux.receive("TELEOP", 0.0, 0.0)
        self.assertEqual(self.mux.mode, "AUTO")
        self.assertEqual(self.mux.output(), (0.5, -1.0))

    def test_human_takeover_disables_auto_until_explicit_mode_change(self):
        self.mux.set_mode("AUTO")
        self.mux.receive("AUTO", 0.4, 0.8)
        self.mux.receive("TELEOP", 0.2, -0.3)
        self.assertEqual(self.mux.mode, "TELEOP")
        self.mux.receive("AUTO", 0.5, 1.0)
        self.assertEqual(self.mux.output(), (0.2, -0.3))
        self.mux.receive("TELEOP", 0.0, 0.0)
        self.assertEqual(self.mux.mode, "TELEOP")
        self.assertEqual(self.mux.output(), (0.0, 0.0))

    def test_turning_alone_triggers_takeover(self):
        self.mux.set_mode("AUTO")
        self.mux.receive("TELEOP", 0.0, -0.1)
        self.assertEqual(self.mux.mode, "TELEOP")

    def test_timeout_stops_either_active_source(self):
        for mode in ("TELEOP", "AUTO"):
            with self.subTest(mode=mode):
                self.clock.return_value = 10.0
                self.mux.set_mode(mode)
                self.mux.receive(mode, 0.2, 0.3)
                self.assertEqual(self.mux.output(), (0.2, 0.3))
                self.clock.return_value = 10.6
                self.assertEqual(self.mux.output(), (0.0, 0.0))

    def test_mode_change_discards_previous_commands(self):
        self.mux.set_mode("AUTO")
        self.mux.receive("AUTO", 0.4, 0.8)
        self.mux.set_mode("STANDBY")
        self.mux.set_mode("AUTO")
        self.assertEqual(self.mux.output(), (0.0, 0.0))

    def test_nonfinite_commands_discard_previous_command(self):
        self.mux.set_mode("TELEOP")
        for value in (float("nan"), float("inf"), float("-inf")):
            for linear, angular in ((value, 0.1), (0.1, value)):
                with self.subTest(linear=linear, angular=angular):
                    self.mux.receive("TELEOP", 0.2, 0.3)
                    self.mux.receive("TELEOP", linear, angular)
                    self.assertEqual(self.mux.output(), (0.0, 0.0))

    def test_invalid_mode_and_source_are_rejected(self):
        with self.assertRaises(ValueError):
            self.mux.set_mode("UNKNOWN")
        with self.assertRaises(ValueError):
            self.mux.receive("UNKNOWN", 0.0, 0.0)
        self.assertEqual(self.mux.mode, "STANDBY")

    def test_invalid_configuration_is_rejected(self):
        for keyword in ("timeout", "linear_limit", "angular_limit"):
            for value in (0.0, -1.0, float("nan"), float("inf")):
                with self.subTest(keyword=keyword, value=value):
                    with self.assertRaises(ValueError):
                        CommandMux(**{keyword: value})
        with self.assertRaises(ValueError):
            CommandMux(deadband=-0.1)


if __name__ == "__main__":
    unittest.main()
