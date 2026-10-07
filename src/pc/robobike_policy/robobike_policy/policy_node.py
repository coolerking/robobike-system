"""ROS adapter for future SmolVLA inference running outside the ROS environment."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class PolicyNode(Node):
    def __init__(self):
        super().__init__("robobike_policy")
        self.enabled = False
        self.image = None
        self.image_time = None
        self.image_timeout = float(self.declare_parameter("image_timeout", 0.5).value)
        if not math.isfinite(self.image_timeout) or self.image_timeout <= 0:
            raise ValueError("image_timeout must be finite and positive")
        self.publisher = self.create_publisher(Twist, "/policy/cmd_vel", 1)
        enable_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/policy/enable", self.on_enable, enable_qos)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, qos_profile_sensor_data)
        self.create_timer(0.1, self.tick)
        self.get_logger().warning("SmolVLA inference is not implemented; no policy commands are emitted.")

    def on_enable(self, message):
        self.enabled = message.data
        self.image = None
        self.image_time = None

    def on_image(self, image):
        if self.enabled:
            self.image = image
            self.image_time = time.monotonic()

    def infer(self, image):
        # TODO: Call the external ML service, map actions to Twist and enforce limits.
        return None

    def tick(self):
        if not self.enabled or self.image_time is None:
            return
        if time.monotonic() - self.image_time > self.image_timeout:
            return
        command = self.infer(self.image)
        if command is not None:
            self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = PolicyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
