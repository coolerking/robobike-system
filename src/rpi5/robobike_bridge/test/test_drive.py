"""ROS-independent checks for the /cmd_vel -> ROBOBIKE command state machine."""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robobike_bridge.drive import (  # noqa: E402
    ALLOWED_BUTTONS, BT_A_DN_0, BT_A_UP_0, BT_BK, BT_F, BT_S, BT_STR_S, STP_ALL,
    Command, DriveConfig, DriveController, steer_value, target_speed,
)


class HelperTest(unittest.TestCase):
    def test_firmware_ids(self):
        self.assertEqual((STP_ALL, BT_F, BT_S, BT_STR_S, BT_BK, BT_A_UP_0, BT_A_DN_0), (1, 3, 4, 7, 8, 9, 10))
        self.assertNotIn(11, ALLOWED_BUTTONS)  # bt_A_Up starts the motor even when stopped
        self.assertNotIn(5, ALLOWED_BUTTONS)  # bt_L may block the HTTP server

    def test_steer_value_inverts_sign_and_saturates(self):
        self.assertEqual(steer_value(0.5, 1.0), -50)
        self.assertEqual(steer_value(-0.25, 1.0), 25)
        self.assertEqual(steer_value(5.0, 1.0), -100)
        self.assertEqual(steer_value(float("nan"), 1.0), 0)

    def test_target_speed(self):
        self.assertEqual(target_speed(0.25, 0.25, 30), 60)
        self.assertEqual(target_speed(1.0, 0.25, 30), 60)
        self.assertEqual(target_speed(0.0625, 0.25, 30), 30)
        self.assertEqual(target_speed(0.1875, 0.25, 30), 45)

    def test_config_validation(self):
        DriveConfig().validate()
        for kwargs in (
            {"cmd_timeout": 0}, {"cmd_timeout": math.nan}, {"control_period": -1},
            {"start_linear": 0.01, "stop_linear": 0.02}, {"stop_linear": -0.1},
            {"max_angular": math.inf}, {"max_linear": 0}, {"min_drive_speed": 61},
            {"min_drive_speed": -1}, {"stop_retries": -1}, {"rearm_duration": math.nan},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                DriveConfig(**kwargs).validate()


class Harness:
    def __init__(self, **kwargs):
        self.ctl = DriveController(DriveConfig(**kwargs))
        self.now = 100.0

    def cmd(self, v, w=0.0):
        self.ctl.on_cmd_vel(self.now, v, w)

    def step(self, sv_drv=0.0, connected=True, dt=0.05, mot_spd=None, shutdown=False):
        self.now += dt
        return self.ctl.update(self.now, sv_drv, connected, mot_spd=mot_spd, shutdown=shutdown)


class DriveControllerTest(unittest.TestCase):
    def test_disabled_controller_never_sends(self):
        ctl = DriveController(DriveConfig(), enabled=False)
        ctl.on_cmd_vel(1.0, 1.0, 0.0)
        self.assertEqual(ctl.update(1.01, 0.0, True), [])
        self.assertEqual(ctl.state, "DISABLED")

    def test_start_is_sent_once_then_running(self):
        h = Harness()
        h.cmd(0.2)
        self.assertEqual(h.step(), [Command(BT_F)])
        self.assertEqual(h.ctl.state, "STARTING")
        h.cmd(0.2)
        self.assertEqual(h.step(), [])
        h.cmd(0.2)
        h.step(sv_drv=60.0)
        self.assertEqual(h.ctl.state, "RUNNING")

    def test_no_start_without_telemetry_or_command(self):
        h = Harness()
        h.cmd(0.2)
        self.assertEqual(h.step(connected=False), [])
        self.assertEqual(h.step(dt=1.0), [])  # command went stale
        self.assertEqual(h.ctl.state, "STOPPED")

    def running(self, **kwargs):
        h = Harness(**kwargs)
        h.cmd(0.2)
        h.step()
        h.cmd(0.2)
        h.step(sv_drv=60.0)
        return h

    def test_steering_is_sent_on_change_and_resent(self):
        h = self.running()
        h.cmd(0.2, 0.5)
        self.assertEqual(h.step(sv_drv=60.0), [Command(BT_STR_S, -50)])
        h.cmd(0.2, 0.5)
        self.assertEqual(h.step(sv_drv=60.0), [])
        h.cmd(0.2, 0.4)
        self.assertEqual(h.step(sv_drv=60.0), [Command(BT_STR_S, -40)])
        outs = []
        for _ in range(10):
            h.cmd(0.2, 0.4)
            outs.append(h.step(sv_drv=60.0))
        self.assertEqual(outs[:-1], [[]] * 9)
        self.assertEqual(outs[-1], [Command(BT_STR_S, -40)])  # periodic resend after 0.5 s

    def test_steering_respects_min_interval(self):
        h = self.running()
        h.cmd(0.2, 0.5)
        h.step(sv_drv=60.0)
        h.cmd(0.2, 0.1)
        self.assertEqual(h.step(sv_drv=60.0, dt=0.02), [])
        h.cmd(0.2, 0.1)
        self.assertEqual(h.step(sv_drv=60.0, dt=0.1), [Command(BT_STR_S, -10)])

    def test_zero_command_stops_and_confirms(self):
        h = self.running()
        h.cmd(0.0)
        self.assertEqual(h.step(sv_drv=60.0), [Command(BT_S)])
        self.assertEqual(h.ctl.state, "STOPPING")
        h.cmd(0.0)
        h.step(sv_drv=0.0)
        self.assertEqual(h.ctl.state, "STOPPED")

    def test_command_timeout_and_lost_telemetry_stop(self):
        h = self.running()
        self.assertEqual(h.step(sv_drv=60.0, dt=0.6), [Command(BT_S)])
        h = self.running()
        h.cmd(0.2)
        self.assertEqual(h.step(sv_drv=60.0, connected=False), [Command(BT_S)])

    def test_shutdown_stops(self):
        h = self.running()
        h.cmd(0.2)
        self.assertEqual(h.step(sv_drv=60.0, shutdown=True), [Command(BT_S)])

    def test_stop_retries_then_stp_all(self):
        h = self.running(stop_retries=2, stop_confirm_timeout=1.0)
        h.cmd(0.0)
        self.assertEqual(h.step(sv_drv=60.0), [Command(BT_S)])
        self.assertEqual(h.step(sv_drv=60.0, dt=1.1), [Command(BT_S)])
        self.assertEqual(h.step(sv_drv=60.0, dt=1.1), [Command(BT_S)])
        self.assertEqual(h.step(sv_drv=60.0, dt=1.1), [Command(STP_ALL)])
        self.assertEqual(h.ctl.state, "FAULT")
        self.assertEqual(h.step(sv_drv=0.0, dt=5.0), [])

    def test_external_stop_locks_out_until_rearmed(self):
        h = self.running()
        h.cmd(0.2)
        self.assertEqual(h.step(sv_drv=0.0), [Command(BT_STR_S, 0)])
        self.assertEqual(h.ctl.state, "LOCKOUT")
        h.cmd(0.2)
        self.assertEqual(h.step(), [])
        for _ in range(10):
            h.cmd(0.0)
            h.step(dt=0.1)
        self.assertEqual(h.ctl.state, "LOCKOUT")
        h.cmd(0.0)
        h.step(dt=0.1)
        self.assertEqual(h.ctl.state, "STOPPED")
        h.cmd(0.2)
        self.assertEqual(h.step(), [Command(BT_F)])

    def test_start_timeout_locks_out(self):
        h = Harness(start_confirm_timeout=1.0)
        h.cmd(0.2)
        h.step()
        h.cmd(0.2)
        h.step(dt=1.1)
        self.assertEqual(h.ctl.state, "LOCKOUT")

    def test_stop_requested_while_starting_waits_for_motor(self):
        h = Harness()
        h.cmd(0.2)
        h.step()
        h.cmd(0.0)
        self.assertEqual(h.step(), [])
        h.cmd(0.0)
        self.assertEqual(h.step(sv_drv=60.0), [Command(BT_S)])

    def test_externally_started_run_is_stopped(self):
        h = Harness()
        h.cmd(0.0)
        self.assertEqual(h.step(sv_drv=60.0), [Command(BT_S)])

    def test_reverse_only_when_allowed(self):
        h = Harness()
        h.cmd(-0.2)
        self.assertEqual(h.step(), [])
        h = Harness(allow_reverse=True)
        h.cmd(-0.2)
        self.assertEqual(h.step(), [Command(BT_BK)])
        h.cmd(-0.2)
        h.step(sv_drv=-20.0)
        h.cmd(0.0)
        self.assertEqual(h.step(sv_drv=-20.0), [Command(BT_BK)])
        self.assertEqual(h.ctl.state, "STOPPING")

    def test_speed_control_steps_and_restores(self):
        h = Harness(speed_control=True)
        h.cmd(0.2)
        h.step(mot_spd=60)
        h.cmd(0.125)
        h.step(sv_drv=60.0, mot_spd=60)
        h.cmd(0.125)
        self.assertEqual(h.step(sv_drv=60.0, mot_spd=60), [Command(BT_A_DN_0)])
        h.cmd(0.125)
        self.assertEqual(h.step(sv_drv=59.0, mot_spd=59), [Command(BT_A_DN_0)])
        h.cmd(0.0)
        self.assertEqual(h.step(sv_drv=58.0, mot_spd=58), [Command(BT_S)])
        h.cmd(0.0)
        h.step(sv_drv=0.0, mot_spd=58)
        h.cmd(0.0)
        self.assertEqual(h.step(sv_drv=0.0, mot_spd=58), [Command(BT_A_UP_0)])
        h.cmd(0.0)
        self.assertEqual(h.step(sv_drv=0.0, mot_spd=59), [Command(BT_A_UP_0)])
        h.cmd(0.0)
        self.assertEqual(h.step(sv_drv=0.0, mot_spd=60), [])

    def test_speed_control_disabled_sends_no_speed_commands(self):
        h = self.running()
        h.cmd(0.06)
        out = h.step(sv_drv=60.0, mot_spd=60)
        self.assertNotIn(Command(BT_A_DN_0), out)


if __name__ == "__main__":
    unittest.main()
