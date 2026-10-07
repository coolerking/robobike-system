"""Publish the only robot command stream and gate autonomous inference."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

from robobike_control.mux import CommandMux


class ControlNode(Node):
    def __init__(self):
        super().__init__("robobike_control")
        self.mux = CommandMux(
            timeout=float(self.declare_parameter("command_timeout", 0.5).value),
            linear_limit=float(self.declare_parameter("linear_limit", 0.5).value),
            angular_limit=float(self.declare_parameter("angular_limit", 1.0).value),
            deadband=float(self.declare_parameter("human_deadband", 0.05).value),
        )
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 1)
        self.enable_publisher = self.create_publisher(Bool, "/policy/enable", latched)
        self.state_publisher = self.create_publisher(String, "/control/state", latched)
        self.create_subscription(String, "/control/mode", self.on_mode, 1)
        self.create_subscription(Twist, "/teleop/cmd_vel", self.on_teleop, 1)
        self.create_subscription(Twist, "/policy/cmd_vel", self.on_policy, 1)
        self.create_timer(0.05, self.publish_command)
        self.publish_state()

    def publish_state(self):
        self.enable_publisher.publish(Bool(data=self.mux.mode == "AUTO"))
        self.state_publisher.publish(String(data=self.mux.mode))

    def on_mode(self, message):
        try:
            self.mux.set_mode(message.data)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        self.publish_state()
        self.publish_command()

    def on_teleop(self, command):
        previous = self.mux.mode
        self.mux.receive("TELEOP", command.linear.x, command.angular.z)
        if self.mux.mode != previous:
            self.get_logger().warning("Human input detected: AUTO -> TELEOP")
            self.publish_state()
            self.publish_command()

    def on_policy(self, command):
        self.mux.receive("AUTO", command.linear.x, command.angular.z)

    def publish_command(self):
        command = Twist()
        command.linear.x, command.angular.z = self.mux.output()
        self.publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mux.set_mode("STANDBY")
        node.publish_state()
        node.publish_command()
        node.destroy_node()
        rclpy.shutdown()
