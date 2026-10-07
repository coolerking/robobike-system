"""Publish frames from a USB camera."""

import math

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraNode(Node):
    def __init__(self):
        super().__init__("pi5_camera")
        device = self.declare_parameter("device", "/dev/video0").value
        fps = float(self.declare_parameter("fps", 15.0).value)
        self.frame_id = self.declare_parameter("frame_id", "camera").value
        if not math.isfinite(fps) or fps <= 0:
            raise ValueError("fps must be finite and positive")
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, "/camera/image_raw", qos_profile_sensor_data)
        self.capture = cv2.VideoCapture(device)
        if not self.capture.isOpened():
            self.capture.release()
            raise RuntimeError(f"Cannot open camera: {device}")
        self.create_timer(1.0 / fps, self.publish_frame)

    def publish_frame(self):
        ok, frame = self.capture.read()
        if not ok:
            self.get_logger().warning("Camera frame unavailable", throttle_duration_sec=5.0)
            return
        message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        self.publisher.publish(message)

    def destroy_node(self):
        self.capture.release()
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
