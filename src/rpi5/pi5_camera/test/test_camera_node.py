"""Checks for the USB camera publisher; runs with or without ROS 2 / OpenCV installed."""

import dataclasses
import importlib
import math
import os
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def _stub_module(name, **attributes):
    module = types.ModuleType(name)
    module.__dict__.update(attributes)
    module._robobike_stub = True
    sys.modules[name] = module
    return module


def _ensure_module(name, build_stub):
    try:
        importlib.import_module(name)
    except ImportError:
        build_stub()


def _stub_cv2():
    _stub_module("cv2", VideoCapture=MagicMock(name="VideoCapture"))


def _stub_cv_bridge():
    class CvBridge:
        def cv2_to_imgmsg(self, frame, encoding="passthrough"):
            return sys.modules["sensor_msgs.msg"].Image()

    _stub_module("cv_bridge", CvBridge=CvBridge)


def _stub_sensor_msgs():
    @dataclasses.dataclass
    class Time:
        sec: int = 0
        nanosec: int = 0

    @dataclasses.dataclass
    class Header:
        stamp: Time = dataclasses.field(default_factory=Time)
        frame_id: str = ""

    @dataclasses.dataclass
    class Image:
        header: Header = dataclasses.field(default_factory=Header)

    package = _stub_module("sensor_msgs")
    package.msg = _stub_module("sensor_msgs.msg", Image=Image)


def _stub_rclpy():
    class Parameter:
        def __init__(self, name, type_=None, value=None):
            self.name = name
            self.value = value

    class Node:
        def __init__(self, node_name, *, parameter_overrides=None, **kwargs):
            self._overrides = {p.name: p.value for p in parameter_overrides or ()}

        def declare_parameter(self, name, value=None, *args, **kwargs):
            return types.SimpleNamespace(value=self._overrides.get(name, value))

        def create_publisher(self, *args, **kwargs):
            return MagicMock(name="publisher")

        def create_timer(self, *args, **kwargs):
            return MagicMock(name="timer")

        def get_logger(self):
            return MagicMock(name="logger")

        def get_clock(self):
            return MagicMock(name="clock")

        def destroy_node(self):
            return True

    package = _stub_module("rclpy", init=lambda *a, **k: None, shutdown=lambda *a, **k: None, spin=lambda *a, **k: None)
    package.node = _stub_module("rclpy.node", Node=Node)
    package.parameter = _stub_module("rclpy.parameter", Parameter=Parameter)
    package.qos = _stub_module("rclpy.qos", qos_profile_sensor_data=object())


_ensure_module("cv2", _stub_cv2)
_ensure_module("sensor_msgs.msg", _stub_sensor_msgs)
_ensure_module("cv_bridge", _stub_cv_bridge)
_ensure_module("rclpy", _stub_rclpy)

import rclpy  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402

from pi5_camera import camera_node  # noqa: E402

REAL_RCLPY = not getattr(rclpy, "_robobike_stub", False)


def setUpModule():
    if REAL_RCLPY:
        rclpy.init()


def tearDownModule():
    if REAL_RCLPY:
        rclpy.shutdown()


class ByIdDirMixin:
    """Provides a fake /dev/v4l/by-id directory and /dev/videoN device files."""

    def setUp(self):
        super().setUp()
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        self.by_id = self.root / "by-id"
        self.by_id.mkdir()
        dir_patch = patch.object(camera_node, "BY_ID_DIR", str(self.by_id))
        dir_patch.start()
        self.addCleanup(dir_patch.stop)

    def make_device(self, name):
        device = self.root / name
        device.touch()
        return str(device)

    def link(self, name, target):
        path = self.by_id / name
        path.symlink_to(target)
        return str(path)


class ResolveDeviceTest(ByIdDirMixin, unittest.TestCase):
    def test_by_id_symlink_is_resolved_to_real_device(self):
        video2 = self.make_device("video2")
        link = self.link("usb-Cam_ABC-video-index0", video2)
        self.assertEqual(camera_node.resolve_device(link), os.path.realpath(video2))

    def test_existing_plain_device_path_is_kept(self):
        video0 = self.make_device("video0")
        self.assertEqual(camera_node.resolve_device(video0), os.path.realpath(video0))

    def test_missing_path_falls_back_to_first_video_index0(self):
        self.link("usb-AAA-video-index1", self.make_device("video1"))
        self.link("usb-BBB-video-index0", self.make_device("video4"))
        self.link("usb-CCC-video-index0", self.make_device("video6"))
        resolved = camera_node.resolve_device(str(self.by_id / "usb-Gone-video-index0"))
        self.assertEqual(resolved, os.path.realpath(self.root / "video4"))

    def test_by_id_directory_param_selects_first_video_index0(self):
        self.link("usb-BBB-video-index0", self.make_device("video3"))
        self.assertEqual(camera_node.resolve_device(str(self.by_id)), os.path.realpath(self.root / "video3"))

    def test_camera_id_selects_partially_matching_entry(self):
        self.link("usb-AAA_Webcam-video-index0", self.make_device("video0"))
        self.link("usb-Logitech_C920_1234-video-index1", self.make_device("video3"))
        self.link("usb-Logitech_C920_1234-video-index0", self.make_device("video2"))
        resolved = camera_node.resolve_device("/dev/v4l/by-id/missing", camera_id="C920")
        self.assertEqual(resolved, os.path.realpath(self.root / "video2"))

    def test_camera_id_without_match_returns_param(self):
        self.link("usb-AAA_Webcam-video-index0", self.make_device("video0"))
        self.assertEqual(camera_node.resolve_device("/no/such/video9", camera_id="C920"), "/no/such/video9")

    def test_dangling_by_id_links_are_ignored(self):
        self.link("usb-AAA-video-index0", str(self.root / "unplugged"))
        self.link("usb-BBB-video-index0", self.make_device("video5"))
        self.assertEqual(camera_node.resolve_device("/no/such"), os.path.realpath(self.root / "video5"))

    def test_without_candidates_param_value_is_used(self):
        self.assertEqual(camera_node.resolve_device("/no/such/video7"), "/no/such/video7")

    def test_by_id_default_without_candidates_falls_back_to_video0(self):
        self.assertEqual(camera_node.resolve_device(str(self.by_id)), "/dev/video0")
        with patch.object(camera_node, "BY_ID_DIR", str(self.root / "absent")):
            self.assertEqual(camera_node.resolve_device(str(self.root / "absent")), "/dev/video0")


class CameraNodeTest(ByIdDirMixin, unittest.TestCase):
    def setUp(self):
        super().setUp()
        self.capture = MagicMock(name="capture")
        self.capture.isOpened.return_value = True
        self.capture.read.return_value = (True, "frame")
        self.logger = MagicMock(name="logger")
        self.clock = MagicMock(name="clock")
        self.patches = {
            "VideoCapture": patch.object(camera_node.cv2, "VideoCapture", return_value=self.capture),
            "create_timer": patch.object(camera_node.CameraNode, "create_timer"),
            "create_publisher": patch.object(camera_node.CameraNode, "create_publisher"),
            "get_logger": patch.object(camera_node.CameraNode, "get_logger", return_value=self.logger),
            "get_clock": patch.object(camera_node.CameraNode, "get_clock", return_value=self.clock),
        }
        self.mocks = {name: p.start() for name, p in self.patches.items()}
        for p in self.patches.values():
            self.addCleanup(p.stop)
        self.video0 = self.make_device("video0")

    def make_node(self, **params):
        params.setdefault("device", self.video0)
        overrides = [Parameter(name, value=value) for name, value in params.items()]
        node = camera_node.CameraNode(parameter_overrides=overrides)
        self.addCleanup(node.destroy_node)
        return node

    def test_invalid_fps_is_rejected(self):
        for fps in (0.0, -1.0, -15.0, float("nan"), float("inf")):
            with self.subTest(fps=fps):
                with self.assertRaises(ValueError):
                    self.make_node(fps=fps)
        self.mocks["VideoCapture"].assert_not_called()

    def test_invalid_reopen_threshold_is_rejected(self):
        with self.assertRaises(ValueError):
            self.make_node(reopen_after_failures=0)

    def test_timer_period_is_inverse_of_fps(self):
        for fps in (15.0, 30.0, 2.5):
            with self.subTest(fps=fps):
                self.mocks["create_timer"].reset_mock()
                node = self.make_node(fps=fps)
                period, callback = self.mocks["create_timer"].call_args.args
                self.assertTrue(math.isclose(period, 1.0 / fps))
                self.assertEqual(callback, node.publish_frame)

    def test_default_fps_and_topic(self):
        self.make_node()
        period, _ = self.mocks["create_timer"].call_args.args
        self.assertTrue(math.isclose(period, 1.0 / 15.0))
        self.assertEqual(self.mocks["create_publisher"].call_args.args[1], "/camera/image_raw")

    def test_topic_parameter_is_used(self):
        self.make_node(topic="/front/image_raw")
        self.assertEqual(self.mocks["create_publisher"].call_args.args[1], "/front/image_raw")

    def test_by_id_device_is_opened_by_real_path(self):
        video2 = self.make_device("video2")
        link = self.link("usb-Cam_ABC-video-index0", video2)
        node = self.make_node(device=link)
        self.mocks["VideoCapture"].assert_called_once_with(os.path.realpath(video2))
        self.assertEqual(node.device, os.path.realpath(video2))

    def test_missing_by_id_falls_back_to_camera_id_match(self):
        self.link("usb-AAA-video-index0", self.make_device("video1"))
        self.link("usb-Logitech_C920-video-index0", self.make_device("video3"))
        self.make_node(device=str(self.by_id / "usb-Unplugged-video-index0"), camera_id="Logitech")
        self.mocks["VideoCapture"].assert_called_once_with(os.path.realpath(self.root / "video3"))

    def test_default_device_scans_by_id_directory(self):
        self.link("usb-AAA-video-index0", self.make_device("video8"))
        with patch.object(camera_node, "DEFAULT_DEVICE", str(self.by_id)):
            overrides = [Parameter("fps", value=15.0)]
            node = camera_node.CameraNode(parameter_overrides=overrides)
            self.addCleanup(node.destroy_node)
        self.mocks["VideoCapture"].assert_called_once_with(os.path.realpath(self.root / "video8"))

    def test_read_failure_does_not_publish(self):
        node = self.make_node()
        self.capture.read.return_value = (False, None)
        node.publish_frame()
        self.mocks["create_publisher"].return_value.publish.assert_not_called()
        self.logger.warning.assert_called()

    def test_successful_read_publishes_with_header(self):
        node = self.make_node(frame_id="front_camera")
        message = camera_node.Image()
        node.bridge = MagicMock()
        node.bridge.cv2_to_imgmsg.return_value = message
        stamp = type(message.header.stamp)(sec=123, nanosec=456)
        self.clock.now.return_value.to_msg.return_value = stamp
        node.publish_frame()
        node.bridge.cv2_to_imgmsg.assert_called_once_with("frame", encoding="bgr8")
        publisher = self.mocks["create_publisher"].return_value
        publisher.publish.assert_called_once_with(message)
        self.assertEqual(message.header.frame_id, "front_camera")
        self.assertEqual(message.header.stamp, stamp)

    def test_open_failure_warns_and_continues(self):
        self.capture.isOpened.return_value = False
        node = self.make_node()
        self.capture.release.assert_called()
        self.logger.warning.assert_called()
        self.mocks["create_timer"].assert_called_once()
        node.publish_frame()
        self.mocks["create_publisher"].return_value.publish.assert_not_called()

    def test_repeated_open_failure_retries_open(self):
        self.capture.isOpened.return_value = False
        node = self.make_node(reopen_after_failures=3)
        self.assertEqual(self.mocks["VideoCapture"].call_count, 1)
        for _ in range(3):
            node.publish_frame()
        self.assertEqual(self.mocks["VideoCapture"].call_count, 2)

    def test_repeated_read_failure_reopens_capture(self):
        node = self.make_node(reopen_after_failures=3)
        self.capture.read.return_value = (False, None)
        for _ in range(2):
            node.publish_frame()
        self.assertEqual(self.mocks["VideoCapture"].call_count, 1)
        node.publish_frame()
        self.assertEqual(self.mocks["VideoCapture"].call_count, 2)
        self.capture.release.assert_called()

    def test_successful_read_resets_failure_count(self):
        node = self.make_node(reopen_after_failures=3)
        for ok in (False, False, True, False, False):
            self.capture.read.return_value = (ok, "frame" if ok else None)
            node.publish_frame()
        self.assertEqual(self.mocks["VideoCapture"].call_count, 1)

    def test_reopen_resolves_device_again(self):
        first = self.make_device("video2")
        self.link("usb-Cam-video-index0", first)
        self.capture.isOpened.return_value = False
        node = self.make_node(device=str(self.by_id), reopen_after_failures=1)
        (self.by_id / "usb-Cam-video-index0").unlink()
        second = self.make_device("video5")
        self.link("usb-Cam-video-index0", second)
        node.publish_frame()
        self.assertEqual(self.mocks["VideoCapture"].call_args.args[0], os.path.realpath(second))


if __name__ == "__main__":
    unittest.main()
