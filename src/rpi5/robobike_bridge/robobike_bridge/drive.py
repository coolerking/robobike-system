"""ROS-independent state machine turning /cmd_vel into discrete ROBOBIKE commands."""

import dataclasses
import math
from collections import namedtuple

# TcmdID values from the firmware (userdefine.h COMMAND_LIST, unknown = 0).
STP_ALL = 1
ONLY_DATA = 2
BT_F = 3
BT_S = 4
BT_STR_S = 7
BT_BK = 8
BT_A_UP_0 = 9
BT_A_DN_0 = 10
# bt_L/bt_R (5/6) can block the HTTP server, bt_A_Up/bt_A_Dn (11/12) start the motor, 13+ change settings.
ALLOWED_BUTTONS = frozenset({STP_ALL, ONLY_DATA, BT_F, BT_S, BT_STR_S, BT_BK, BT_A_UP_0, BT_A_DN_0})
MAX_MOT_SPD = 60
EPS = 1e-9

DISABLED, STOPPED, STARTING, RUNNING, STOPPING, REVERSE, LOCKOUT, FAULT = (
    "DISABLED", "STOPPED", "STARTING", "RUNNING", "STOPPING", "REVERSE", "LOCKOUT", "FAULT",
)

Command = namedtuple("Command", "button value", defaults=(None,))


def _clamp(value, low, high):
    return max(low, min(high, value))


def steer_value(angular_z, max_angular):
    """ROS +angular.z turns left, ROBOBIKE +value steers right: invert and scale to -100..100."""
    if not math.isfinite(angular_z):
        return 0
    return int(round(_clamp(-angular_z / max_angular, -1.0, 1.0) * 100))


def target_speed(linear_x, max_linear, min_drive_speed):
    return int(_clamp(round(linear_x / max_linear * MAX_MOT_SPD), min_drive_speed, MAX_MOT_SPD))


@dataclasses.dataclass
class DriveConfig:
    cmd_timeout: float = 0.5
    control_period: float = 0.05
    start_linear: float = 0.05
    stop_linear: float = 0.02
    max_angular: float = 1.0
    steer_min_interval: float = 0.1
    steer_resend_period: float = 0.5
    allow_reverse: bool = False
    speed_control: bool = False
    max_linear: float = 0.25
    min_drive_speed: int = 30
    start_confirm_timeout: float = 5.0
    stop_confirm_timeout: float = 2.0
    stop_retries: int = 3
    rearm_duration: float = 1.0

    def validate(self):
        for name in (
            "cmd_timeout", "control_period", "start_linear", "max_angular", "steer_resend_period",
            "max_linear", "start_confirm_timeout", "stop_confirm_timeout",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        for name in ("stop_linear", "steer_min_interval", "rearm_duration"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and non-negative")
        if self.stop_linear >= self.start_linear:
            raise ValueError("stop_linear must be smaller than start_linear")
        if not 0 <= int(self.min_drive_speed) <= MAX_MOT_SPD:
            raise ValueError(f"min_drive_speed must be within 0..{MAX_MOT_SPD}")
        if int(self.stop_retries) < 0:
            raise ValueError("stop_retries must be non-negative")
        return self


class DriveController:
    """Evaluate once per control period; the real motion state always comes from telemetry SV_DRV."""

    def __init__(self, config, enabled=True):
        self.config = config.validate()
        self.state = STOPPED if enabled else DISABLED
        self.state_since = 0.0
        self.last_cmd = None  # (time, linear, angular)
        self.wants_stop_while_starting = False
        self.reverse_confirmed = False
        self.stop_attempts = 0
        self.rearm_since = None
        self.steer_value = 0
        self.steer_sent_at = None
        self.steer_ref = 0.0
        self.initial_mot_spd = None
        self.last_command = None
        self.suspected_drops = 0  # start/stop not confirmed by telemetry in time

    def on_cmd_vel(self, now, linear_x, angular_z):
        self.last_cmd = (now, float(linear_x), float(angular_z))

    def _command(self, now):
        if self.last_cmd is None or now - self.last_cmd[0] > self.config.cmd_timeout:
            return 0.0, 0.0, True
        linear, angular = self.last_cmd[1], self.last_cmd[2]
        return (linear if math.isfinite(linear) else 0.0), angular, False

    def _enter(self, state, now):
        self.state, self.state_since = state, now
        if state == RUNNING:
            self.steer_value, self.steer_sent_at, self.steer_ref = 0, None, now
        elif state == STOPPING:
            self.stop_attempts = 0
        elif state == LOCKOUT:
            self.rearm_since = None
        elif state == REVERSE:
            self.reverse_confirmed = False
        elif state == STARTING:
            self.wants_stop_while_starting = False

    def update(self, now, sv_drv, connected, mot_spd=None, shutdown=False):
        """Return the commands to send now (possibly empty)."""
        if self.state == DISABLED:
            return []
        if mot_spd is not None and self.initial_mot_spd is None:
            self.initial_mot_spd = int(mot_spd)
        commands = self._step(now, 0.0 if sv_drv is None else sv_drv, connected, mot_spd, shutdown)
        if commands:
            self.last_command = commands[-1]
        return commands

    def _step(self, now, sv_drv, connected, mot_spd, shutdown):
        cfg = self.config
        v, w, stale = self._command(now)
        state = self.state
        if state == STOPPED:
            if sv_drv > 0:  # started from the phone: ROS has authority, handle as running
                self._enter(RUNNING, now)
                state = RUNNING
            elif sv_drv < 0:
                self._enter(REVERSE, now)
                self.reverse_confirmed = True
                state = REVERSE
            elif shutdown:
                return []
            elif connected and v > cfg.start_linear:
                self._enter(STARTING, now)
                return [Command(BT_F)]
            elif connected and cfg.allow_reverse and v < -cfg.start_linear:
                self._enter(REVERSE, now)
                return [Command(BT_BK)]
            else:
                return self._restore_speed(mot_spd) if connected else []

        if state == STARTING:
            wants_stop = shutdown or not connected or stale or v < cfg.stop_linear
            if sv_drv > 0:
                if wants_stop or self.wants_stop_while_starting:
                    self._enter(STOPPING, now)
                    return [Command(BT_S)]
                self._enter(RUNNING, now)
                return []
            if wants_stop:
                self.wants_stop_while_starting = True
            if now - self.state_since + EPS >= cfg.start_confirm_timeout:
                self.suspected_drops += 1
                if self.wants_stop_while_starting:
                    self._enter(STOPPING, now)
                    return [Command(BT_S)]
                self._enter(LOCKOUT, now)
            return []

        if state == RUNNING:
            if shutdown or not connected or stale or v < cfg.stop_linear:
                self._enter(STOPPING, now)
                return [Command(BT_S)]
            if sv_drv <= 0:  # stopped without us asking: phone STOP or fall detection
                self._enter(LOCKOUT, now)
                return [Command(BT_STR_S, 0)]
            return self._steer(now, w) + self._speed(v, mot_spd)

        if state == REVERSE:
            if sv_drv < 0:
                self.reverse_confirmed = True
            if shutdown or not connected or stale or v > -cfg.stop_linear:
                if sv_drv < 0 or not self.reverse_confirmed:
                    self._enter(STOPPING, now)
                    return [Command(BT_BK)]
            if sv_drv == 0:
                if self.reverse_confirmed or now - self.state_since + EPS >= cfg.start_confirm_timeout:
                    self._enter(LOCKOUT, now)
            return []

        if state == STOPPING:
            if sv_drv == 0:
                self._enter(STOPPED if v < cfg.stop_linear or shutdown else LOCKOUT, now)
                return []
            if now - self.state_since + EPS >= cfg.stop_confirm_timeout:
                self.state_since = now
                self.suspected_drops += 1
                if self.stop_attempts < cfg.stop_retries:
                    self.stop_attempts += 1
                    return [Command(BT_S)]
                self.state = FAULT
                return [Command(STP_ALL)]
            return []

        if state == LOCKOUT:
            if sv_drv > 0:
                self._enter(RUNNING, now)
                return self._step(now, sv_drv, connected, mot_spd, shutdown)
            if v < cfg.stop_linear:
                if self.rearm_since is None:
                    self.rearm_since = now
                if now - self.rearm_since + EPS >= cfg.rearm_duration:
                    self._enter(STOPPED, now)
            else:
                self.rearm_since = None
            return []

        return []  # FAULT: wait for a restart

    def _steer(self, now, angular):
        cfg = self.config
        value = steer_value(angular, cfg.max_angular)
        since_sent = None if self.steer_sent_at is None else now - self.steer_sent_at
        changed = value != self.steer_value and (since_sent is None or since_sent + EPS >= cfg.steer_min_interval)
        resend = now - (self.steer_sent_at if self.steer_sent_at is not None else self.steer_ref) + EPS >= (
            cfg.steer_resend_period
        )
        if not (changed or resend):
            return []
        self.steer_value, self.steer_sent_at = value, now
        return [Command(BT_STR_S, value)]

    def _speed(self, v, mot_spd):
        cfg = self.config
        if not cfg.speed_control or mot_spd is None:
            return []
        target = target_speed(v, cfg.max_linear, cfg.min_drive_speed)
        return self._step_speed(mot_spd, target)

    def _restore_speed(self, mot_spd):
        if not self.config.speed_control or mot_spd is None or self.initial_mot_spd is None:
            return []
        return self._step_speed(mot_spd, self.initial_mot_spd)

    @staticmethod
    def _step_speed(current, target):
        if current < target:
            return [Command(BT_A_UP_0)]
        if current > target:
            return [Command(BT_A_DN_0)]
        return []
