"""Hardware transport extension point; deliberately does not actuate motors."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class BridgeNode(Node):
    def __init__(self):
        super().__init__("robobike_bridge")
        self.create_subscription(Twist, "/cmd_vel", self.send_command, 1)
        self.get_logger().warning("Hardware transport is not implemented; no motors will be driven.")

    def send_command(self, command):
        # TODO: Map Twist to the hardware protocol, with limits and a local watchdog.
        pass


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
