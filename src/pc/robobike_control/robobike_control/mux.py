"""ROS-independent planar velocity arbitration."""

import math
import time


class CommandMux:
    MODES = ("STANDBY", "TELEOP", "AUTO")

    def __init__(self, timeout=0.5, linear_limit=0.5, angular_limit=1.0, deadband=0.05):
        if not all(math.isfinite(v) and v > 0 for v in (timeout, linear_limit, angular_limit)):
            raise ValueError("Timeout and velocity limits must be finite and positive")
        if not math.isfinite(deadband) or deadband < 0:
            raise ValueError("Deadband must be finite and non-negative")
        self.timeout = timeout
        self.linear_limit = linear_limit
        self.angular_limit = angular_limit
        self.deadband = deadband
        self.mode = "STANDBY"
        self.commands = {}

    def set_mode(self, mode):
        if mode not in self.MODES:
            raise ValueError(f"Unknown mode: {mode}")
        if mode != self.mode:
            self.commands.clear()
            self.mode = mode

    def receive(self, source, linear, angular):
        if source not in ("TELEOP", "AUTO"):
            raise ValueError(f"Unknown source: {source}")
        if not math.isfinite(linear) or not math.isfinite(angular):
            self.commands.pop(source, None)
            return
        if source == "TELEOP" and self.mode == "AUTO":
            if abs(linear) > self.deadband or abs(angular) > self.deadband:
                self.set_mode("TELEOP")
        if source == self.mode:
            self.commands[source] = (
                max(-self.linear_limit, min(self.linear_limit, linear)),
                max(-self.angular_limit, min(self.angular_limit, angular)),
                time.monotonic(),
            )

    def output(self):
        command = self.commands.get(self.mode)
        if command is None or time.monotonic() - command[2] > self.timeout:
            return 0.0, 0.0
        return command[:2]
