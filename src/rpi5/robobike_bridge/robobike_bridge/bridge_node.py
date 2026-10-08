"""Bridge between ROBOBIKE's HTTP API and ROS 2: telemetry up, /cmd_vel down (see doc/spec/robobike-bridge.md)."""

import math
import queue
import signal
import threading
import time

import rclpy
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robobike_msgs.msg import RobobikeTelemetry
from std_msgs.msg import Bool, String

from .drive import DISABLED, FAULT, LOCKOUT, STARTING, STOPPED, Command, DriveConfig, DriveController
from .http_client import RobobikeHttpClient, RobobikeHttpError
from .telemetry import ClockMapper, GapDetector, parse_body

__all__ = ["BridgeNode", "Command", "main"]

RING_BUFFER_SECONDS = 5.0
BACKOFF_INITIAL = 0.5
SAMPLE_FIELDS = (
    "sv_drv", "sv_str", "sv_std", "gy_roll", "gy_yaw", "gy_pitch", "swp_sg", "sv_pos", "acc_x", "acc_y", "acc_z",
)
DRIVE_PARAMETERS = (
    ("cmd_timeout", 0.5), ("control_period", 0.05), ("start_linear", 0.05), ("stop_linear", 0.02),
    ("max_angular", 1.0), ("steer_min_interval", 0.1), ("steer_resend_period", 0.5), ("allow_reverse", False),
    ("speed_control", False), ("max_linear", 0.25), ("min_drive_speed", 30), ("start_confirm_timeout", 5.0),
    ("stop_confirm_timeout", 2.0), ("stop_retries", 3), ("rearm_duration", 1.0),
)


def _time_msg(nanoseconds):
    return Time(sec=int(nanoseconds // 1_000_000_000), nanosec=int(nanoseconds % 1_000_000_000))


def _positive(name, value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return value


class BridgeNode(Node):
    def __init__(self, **kwargs):
        super().__init__("robobike_bridge", **kwargs)

        def param(name, default):
            return self.declare_parameter(name, default).value

        self.base_url = str(param("base_url", "http://192.168.4.1"))
        self.poll_period = _positive("poll_period", param("poll_period", 0.05))
        if self.poll_period >= RING_BUFFER_SECONDS:
            raise ValueError(f"poll_period must be shorter than the {RING_BUFFER_SECONDS} s ring buffer")
        self.http_timeout = _positive("http_timeout", param("http_timeout", 0.5))
        self.clear_buffer_on_start = bool(param("clear_buffer_on_start", True))
        self.frame_id = str(param("frame_id", "robobike"))
        self.telemetry_topic = str(param("topic", "/robobike/telemetry"))
        qos_reliable = bool(param("qos_reliable", False))
        qos_depth = int(param("qos_depth", 5))
        if qos_depth < 1:
            raise ValueError("qos_depth must be at least 1")
        self.stale_timeout = _positive("stale_timeout", param("stale_timeout", 1.0))
        self.reconnect_backoff_max = _positive("reconnect_backoff_max", param("reconnect_backoff_max", 5.0))
        self.enable_drive = bool(param("enable_drive", False))
        cmd_topic = str(param("cmd_topic", "/cmd_vel"))
        self.drive_config = DriveConfig(**{name: param(name, default) for name, default in DRIVE_PARAMETERS})
        self.controller = DriveController(self.drive_config, enabled=self.enable_drive)
        self.telemetry_client = RobobikeHttpClient(self.base_url, self.http_timeout)
        self.command_client = RobobikeHttpClient(self.base_url, self.http_timeout)

        self.lock = threading.Lock()
        self.clock = ClockMapper()
        self.gaps = GapDetector()
        self.need_clear = self.clear_buffer_on_start
        self.backoff = 0.0
        self.http_errors = self.malformed = self.samples_total = self.format_changes = 0
        self.columns = None
        self.sv_drv = None
        self.mot_spd = None
        self.last_sample_at = None
        self.command_errors = self.commands_sent = 0
        self.last_sent = None
        self.shutting_down = False
        self.command_queue = queue.Queue()
        self._stop_event = threading.Event()
        self._threads = []
        self._diag_samples = 0
        self._diag_at = time.monotonic()
        self._published_connected = None
        self._published_state = None

        reliability = ReliabilityPolicy.RELIABLE if qos_reliable else ReliabilityPolicy.BEST_EFFORT
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.telemetry_publisher = self.create_publisher(
            RobobikeTelemetry, self.telemetry_topic, QoSProfile(depth=qos_depth, reliability=reliability)
        )
        self.state_publisher = self.create_publisher(String, "/robobike/drive_state", latched)
        self.connected_publisher = self.create_publisher(Bool, "/robobike/bridge/connected", latched)
        self.diagnostics_publisher = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.cmd_subscription = None
        if self.enable_drive:
            self.cmd_subscription = self.create_subscription(
                Twist, cmd_topic, self.on_cmd_vel, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
            )
            self.get_logger().warning(f"Drive enabled: {cmd_topic} will move ROBOBIKE. Keep the phone STOP at hand.")
        else:
            self.get_logger().info("Telemetry only (enable_drive:=false); no commands will be sent.")
        self.create_timer(self.drive_config.control_period, self.control_tick)
        self.create_timer(1.0, self.publish_diagnostics)
        self._publish_state()
        self._publish_connected()

    @property
    def drive_state(self):
        return self.controller.state

    def connected(self):
        with self.lock:
            last = self.last_sample_at
        return last is not None and time.monotonic() - last < self.stale_timeout

    # --- telemetry (worker thread) ---
    def poll_telemetry_once(self):
        """Fetch /get_acc once and publish every sample; return the count, or None on an HTTP error."""
        try:
            if self.need_clear:
                self.telemetry_client.clear_buffer()
                self.need_clear = False
            body = self.telemetry_client.get_acc()
        except RobobikeHttpError as error:
            with self.lock:
                self.http_errors += 1
                self.backoff = min(self.reconnect_backoff_max, self.backoff * 2 if self.backoff else BACKOFF_INITIAL)
            self.need_clear = self.clear_buffer_on_start
            self.get_logger().warning(f"ROBOBIKE telemetry unavailable: {error}", throttle_duration_sec=5.0)
            self._publish_connected()
            return None
        recv_ns = self.get_clock().now().nanoseconds
        samples, malformed = parse_body(body)
        with self.lock:
            self.backoff = 0.0
            self.malformed += malformed
        if not samples:
            self._publish_connected()
            return 0
        if self.clock.observe(recv_ns, samples[-1].time_ms):
            self.get_logger().warning("ROBOBIKE TIME_MS went backwards (reboot?); re-estimating the clock offset")
        for sample in samples:
            self.gaps.feed(sample.time_ms)
            if sample.columns != self.columns:
                if self.columns is not None:
                    self.format_changes += 1
                    self.get_logger().warning(f"Telemetry format changed to {sample.columns} columns")
                self.columns = sample.columns
            self._publish(self.telemetry_publisher, self._telemetry_msg(sample))
        with self.lock:
            self.samples_total += len(samples)
            self.sv_drv = samples[-1].sv_drv
            self.last_sample_at = time.monotonic()
        self._publish_connected()
        return len(samples)

    def _telemetry_msg(self, sample):
        msg = RobobikeTelemetry()
        msg.header.stamp = _time_msg(self.clock.stamp_ns(sample.time_ms))
        msg.header.frame_id = self.frame_id
        msg.record_type = sample.record_type
        msg.time_ms = sample.time_ms & 0xFFFFFFFF
        for name in SAMPLE_FIELDS:
            setattr(msg, name, float(getattr(sample, name)))
        return msg

    # --- drive (executor thread) ---
    def on_cmd_vel(self, msg):
        self.controller.on_cmd_vel(time.monotonic(), msg.linear.x, msg.angular.z)

    def control_tick(self):
        connected = self.connected()
        self._publish_connected(connected)
        if self.controller.state == DISABLED:
            return
        with self.lock:
            sv_drv, mot_spd = self.sv_drv, self.mot_spd
        previous = self.controller.state
        commands = self.controller.update(time.monotonic(), sv_drv, connected, mot_spd, self.shutting_down)
        for command in commands:
            self.command_queue.put(command)
        if self.controller.state != previous:
            log = self.get_logger().error if self.controller.state == FAULT else self.get_logger().info
            log(f"Drive state {previous} -> {self.controller.state}")
        self._publish_state()

    # --- commands (worker thread) ---
    def send_pending_commands(self):
        while True:
            try:
                command = self.command_queue.get_nowait()
            except queue.Empty:
                return
            self._send(command)

    def _send(self, command):
        try:
            settings = self.command_client.command(command.button, command.value)
        except RobobikeHttpError as error:
            with self.lock:
                self.command_errors += 1
            self.get_logger().warning(f"ROBOBIKE command {command} failed: {error}", throttle_duration_sec=5.0)
            return
        with self.lock:
            self.commands_sent += 1
            self.last_sent = command
            if isinstance(settings.get("MOT_SPD"), int):
                self.mot_spd = settings["MOT_SPD"]

    # --- threads and shutdown ---
    def start_workers(self):
        self._stop_event.clear()
        self._threads = [
            threading.Thread(target=self._telemetry_loop, name="robobike_telemetry", daemon=True),
            threading.Thread(target=self._command_loop, name="robobike_command", daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    def stop_workers(self):
        self._stop_event.set()
        for thread in self._threads:
            thread.join(timeout=2.0 * self.http_timeout + 1.0)
        self._threads = []

    def _telemetry_loop(self):
        while not self._stop_event.is_set():
            started = time.monotonic()
            result = self.poll_telemetry_once()
            delay = self.backoff if result is None else self.poll_period - (time.monotonic() - started)
            self._stop_event.wait(max(0.0, delay))

    def _command_loop(self):
        while not self._stop_event.is_set():
            try:
                command = self.command_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            self._send(command)

    def shutdown_drive(self):
        """Stop a moving bike before exiting; wait for telemetry to confirm, bounded by the confirm timeouts."""
        if self.controller.state == DISABLED:
            return
        self.shutting_down = True
        cfg = self.drive_config
        timeout = cfg.stop_confirm_timeout + (cfg.start_confirm_timeout if self.controller.state == STARTING else 0.0)
        deadline = time.monotonic() + timeout
        threaded = bool(self._threads)
        while True:
            if not threaded:
                self.poll_telemetry_once()
            self.control_tick()
            if not threaded:
                self.send_pending_commands()
            if self.controller.state in (STOPPED, LOCKOUT, FAULT) or time.monotonic() >= deadline:
                break
            time.sleep(cfg.control_period)
        if threaded:
            deadline = time.monotonic() + self.http_timeout
            while not self.command_queue.empty() and time.monotonic() < deadline:
                time.sleep(0.01)
        if self.controller.state not in (STOPPED, LOCKOUT, FAULT):
            self.get_logger().error("Could not confirm that ROBOBIKE stopped; use the phone STOP button")

    def destroy_node(self):
        self.stop_workers()
        self.telemetry_client.close()
        self.command_client.close()
        return super().destroy_node()

    # --- publishing ---
    def _publish(self, publisher, msg):
        try:
            publisher.publish(msg)
        except Exception as error:  # context already shut down while stopping
            self.get_logger().debug(f"publish failed: {error}")

    def _publish_state(self):
        state = self.controller.state
        if state != self._published_state:
            self._published_state = state
            msg = String()
            msg.data = state
            self._publish(self.state_publisher, msg)

    def _publish_connected(self, connected=None):
        connected = self.connected() if connected is None else connected
        if connected != self._published_connected:
            self._published_connected = connected
            msg = Bool()
            msg.data = connected
            self._publish(self.connected_publisher, msg)

    def publish_diagnostics(self):
        now = time.monotonic()
        with self.lock:
            samples, last, errors = self.samples_total, self.last_sample_at, self.http_errors
            cmd_errors, sent, mot_spd = self.command_errors, self.last_sent, self.mot_spd
        rate = (samples - self._diag_samples) / max(now - self._diag_at, 1e-6)
        self._diag_samples, self._diag_at = samples, now
        connected = self.connected()
        age = "never" if last is None else f"{now - last:.3f}"

        def status(name, level, message, values):
            return DiagnosticStatus(
                level=level, name=name, message=message, hardware_id=self.base_url,
                values=[KeyValue(key=key, value=str(value)) for key, value in values],
            )

        state = self.controller.state
        drive_level = {FAULT: DiagnosticStatus.ERROR, LOCKOUT: DiagnosticStatus.WARN}.get(state, DiagnosticStatus.OK)
        msg = DiagnosticArray()
        msg.header.stamp = _time_msg(self.get_clock().now().nanoseconds)
        msg.status = [
            status(
                "robobike_bridge: telemetry", DiagnosticStatus.OK if connected else DiagnosticStatus.WARN,
                "receiving" if connected else "no telemetry",
                [("rate_hz", f"{rate:.1f}"), ("missing_samples", self.gaps.missing), ("malformed_lines", self.malformed),
                 ("http_errors", errors), ("columns", self.columns), ("format_changes", self.format_changes),
                 ("clock_rewinds", self.clock.rewinds), ("seconds_since_last_sample", age)],
            ),
            status(
                "robobike_bridge: drive", drive_level, state,
                [("enable_drive", self.enable_drive), ("last_command", sent), ("command_errors", cmd_errors),
                 ("suspected_dropped_commands", self.controller.suspected_drops), ("mot_spd", mot_spd)],
            ),
        ]
        self._publish(self.diagnostics_publisher, msg)


def main(args=None):
    try:
        from rclpy.signals import SignalHandlerOptions

        # Keep the context alive after Ctrl-C so the stop command can still be sent and reported.
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    except ImportError:
        rclpy.init(args=args)

    def terminate(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate)
    node = None
    try:
        node = BridgeNode()
        node.start_workers()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown_drive()
            node.destroy_node()
        rclpy.try_shutdown()
