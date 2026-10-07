"""Publish frames from a USB camera."""

import glob
import math
import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

BY_ID_DIR = "/dev/v4l/by-id"
DEFAULT_DEVICE = BY_ID_DIR
FALLBACK_DEVICE = "/dev/video0"


def _find_by_id(scan_dir, camera_id):
    candidates = [path for path in sorted(glob.glob(os.path.join(scan_dir, "*"))) if os.path.exists(path)]
    capture_nodes = [path for path in candidates if path.endswith("video-index0")]
    if not camera_id:
        return capture_nodes[0] if capture_nodes else None
    matches = [path for path in capture_nodes if camera_id in os.path.basename(path)]
    matches = matches or [path for path in candidates if camera_id in os.path.basename(path)]
    return matches[0] if matches else None


def resolve_device(device_param, camera_id=""):
    """Resolve a stable /dev/v4l/by-id path (or the by-id directory) to the current /dev/videoN."""
    device_param = str(device_param)
    is_dir = os.path.isdir(device_param)
    if os.path.exists(device_param) and not is_dir:
        return os.path.realpath(device_param)
    found = _find_by_id(device_param if is_dir else BY_ID_DIR, camera_id)
    if found:
        return os.path.realpath(found)
    if is_dir or os.path.normpath(device_param) == os.path.normpath(BY_ID_DIR):
        return FALLBACK_DEVICE
    return device_param


class CameraNode(Node):
    def __init__(self, **kwargs):
        super().__init__("pi5_camera", **kwargs)
        self.device_param = str(self.declare_parameter("device", DEFAULT_DEVICE).value)
        self.camera_id = str(self.declare_parameter("camera_id", "").value)
        fps = float(self.declare_parameter("fps", 15.0).value)
        self.frame_id = self.declare_parameter("frame_id", "camera").value
        topic = self.declare_parameter("topic", "/camera/image_raw").value
        self.reopen_after_failures = int(self.declare_parameter("reopen_after_failures", 30).value)
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError("fps must be finite and positive")
        if self.reopen_after_failures < 1:
            raise ValueError("reopen_after_failures must be at least 1")
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, topic, qos_profile_sensor_data)
        self.capture = None
        self.device = None
        self.failures = 0
        self.open_capture()
        self.timer = self.create_timer(1.0 / fps, self.publish_frame)

    def open_capture(self):
        self.release_capture()
        self.device = resolve_device(self.device_param, self.camera_id)
        capture = cv2.VideoCapture(self.device)
        if not capture.isOpened():
            capture.release()
            self.get_logger().warning(f"Cannot open camera: {self.device} (will retry)")
            return False
        self.capture = capture
        self.get_logger().info(f"Opened camera: {self.device}")
        return True

    def release_capture(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None

    def publish_frame(self):
        ok, frame = self.capture.read() if self.capture is not None else (False, None)
        if not ok:
            self.failures += 1
            self.get_logger().warning("Camera frame unavailable", throttle_duration_sec=5.0)
            if self.failures >= self.reopen_after_failures:
                self.failures = 0
                self.open_capture()
            return
        self.failures = 0
        message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        self.publisher.publish(message)

    def destroy_node(self):
        self.release_capture()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
