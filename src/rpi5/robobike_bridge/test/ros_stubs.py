"""Minimal stand-ins for rclpy and message packages, installed only when the real ones are missing."""

import dataclasses
import importlib
import math
import sys
import time
import types
from unittest.mock import MagicMock


def _module(name, **attributes):
    module = types.ModuleType(name)
    module.__dict__.update(attributes)
    module._robobike_stub = True
    sys.modules[name] = module
    if "." in name:
        parent, child = name.rsplit(".", 1)
        setattr(sys.modules[parent], child, module)
    return module


def _missing(name):
    try:
        importlib.import_module(name)
        return False
    except ImportError:
        return True


@dataclasses.dataclass
class Time:
    sec: int = 0
    nanosec: int = 0


@dataclasses.dataclass
class Header:
    stamp: Time = dataclasses.field(default_factory=Time)
    frame_id: str = ""


def _install_rclpy():
    class Parameter:
        def __init__(self, name, type_=None, value=None):
            self.name, self.value = name, value

    class _Clock:
        def now(self):
            return types.SimpleNamespace(nanoseconds=time.time_ns())

    class Node:
        def __init__(self, name, parameter_overrides=None, **kwargs):
            self._name = name
            self._overrides = {p.name: p.value for p in parameter_overrides or []}
            self._logger = MagicMock(name="logger")
            self.subscriptions, self.timers = [], []

        def declare_parameter(self, name, default):
            return types.SimpleNamespace(value=self._overrides.get(name, default))

        def create_publisher(self, msg_type, topic, qos):
            return MagicMock(name=f"publisher:{topic}", topic=topic, msg_type=msg_type, qos=qos)

        def create_subscription(self, msg_type, topic, callback, qos):
            sub = types.SimpleNamespace(msg_type=msg_type, topic=topic, callback=callback, qos=qos)
            self.subscriptions.append(sub)
            return sub

        def create_timer(self, period, callback):
            timer = types.SimpleNamespace(timer_period_ns=int(period * 1e9), callback=callback)
            self.timers.append(timer)
            return timer

        def get_logger(self):
            return self._logger

        def get_clock(self):
            return _Clock()

        def get_name(self):
            return self._name

        def destroy_node(self):
            pass

    class ReliabilityPolicy:
        RELIABLE, BEST_EFFORT = "reliable", "best_effort"

    class DurabilityPolicy:
        TRANSIENT_LOCAL, VOLATILE = "transient_local", "volatile"

    class HistoryPolicy:
        KEEP_LAST = "keep_last"

    @dataclasses.dataclass
    class QoSProfile:
        depth: int = 10
        reliability: str = ReliabilityPolicy.RELIABLE
        durability: str = DurabilityPolicy.VOLATILE
        history: str = HistoryPolicy.KEEP_LAST

    rclpy = _module("rclpy", init=lambda args=None: None, shutdown=lambda: None, ok=lambda: True, spin=lambda node: None)
    _module("rclpy.node", Node=Node)
    _module("rclpy.parameter", Parameter=Parameter)
    _module(
        "rclpy.qos", QoSProfile=QoSProfile, ReliabilityPolicy=ReliabilityPolicy,
        DurabilityPolicy=DurabilityPolicy, HistoryPolicy=HistoryPolicy,
        qos_profile_sensor_data=QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
    )
    return rclpy


def _install_messages():
    if _missing("builtin_interfaces.msg"):
        _module("builtin_interfaces")
        _module("builtin_interfaces.msg", Time=Time)
    if _missing("std_msgs.msg"):
        @dataclasses.dataclass
        class String:
            data: str = ""

        @dataclasses.dataclass
        class Bool:
            data: bool = False

        _module("std_msgs")
        _module("std_msgs.msg", String=String, Bool=Bool, Header=Header)
    if _missing("geometry_msgs.msg"):
        @dataclasses.dataclass
        class Vector3:
            x: float = 0.0
            y: float = 0.0
            z: float = 0.0

        @dataclasses.dataclass
        class Twist:
            linear: Vector3 = dataclasses.field(default_factory=Vector3)
            angular: Vector3 = dataclasses.field(default_factory=Vector3)

        _module("geometry_msgs")
        _module("geometry_msgs.msg", Twist=Twist, Vector3=Vector3)
    if _missing("diagnostic_msgs.msg"):
        @dataclasses.dataclass
        class KeyValue:
            key: str = ""
            value: str = ""

        class DiagnosticStatus:
            OK, WARN, ERROR, STALE = b"\x00", b"\x01", b"\x02", b"\x03"

            def __init__(self, level=b"\x00", name="", message="", hardware_id="", values=None):
                self.level, self.name, self.message = level, name, message
                self.hardware_id, self.values = hardware_id, values or []

        @dataclasses.dataclass
        class DiagnosticArray:
            header: Header = dataclasses.field(default_factory=Header)
            status: list = dataclasses.field(default_factory=list)

        _module("diagnostic_msgs")
        _module("diagnostic_msgs.msg", KeyValue=KeyValue, DiagnosticStatus=DiagnosticStatus, DiagnosticArray=DiagnosticArray)
    if _missing("robobike_msgs.msg"):
        @dataclasses.dataclass
        class RobobikeTelemetry:
            header: Header = dataclasses.field(default_factory=Header)
            record_type: str = ""
            time_ms: int = 0
            sv_drv: float = 0.0
            sv_str: float = 0.0
            sv_std: float = 0.0
            gy_roll: float = 0.0
            gy_yaw: float = 0.0
            gy_pitch: float = 0.0
            swp_sg: float = math.nan
            sv_pos: float = math.nan
            acc_x: float = math.nan
            acc_y: float = math.nan
            acc_z: float = math.nan

        _module("robobike_msgs")
        _module("robobike_msgs.msg", RobobikeTelemetry=RobobikeTelemetry)


def install():
    """Return True when the real rclpy is used."""
    real = not _missing("rclpy")
    if not real:
        _install_rclpy()
    _install_messages()
    return real
