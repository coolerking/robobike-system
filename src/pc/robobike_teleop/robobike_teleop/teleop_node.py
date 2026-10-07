"""Dead-man gated joystick velocity commands."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy


class TeleopNode(Node):
    def __init__(self):
        super().__init__("robobike_teleop")
        self.linear_axis = int(self.declare_parameter("linear_axis", 1).value)
        self.angular_axis = int(self.declare_parameter("angular_axis", 0).value)
        self.deadman_button = int(self.declare_parameter("deadman_button", 0).value)
        self.linear_scale = float(self.declare_parameter("linear_scale", 0.5).value)
        self.angular_scale = float(self.declare_parameter("angular_scale", 1.0).value)
        self.timeout = float(self.declare_parameter("joy_timeout", 0.5).value)
        if min(self.linear_axis, self.angular_axis, self.deadman_button) < 0:
            raise ValueError("Joystick indices must be non-negative")
        if not all(math.isfinite(v) and v > 0 for v in (
            self.linear_scale, self.angular_scale, self.timeout
        )):
            raise ValueError("Scales and timeout must be finite and positive")
        self.last_joy = None
        self.publisher = self.create_publisher(Twist, "/teleop/cmd_vel", 1)
        self.create_subscription(Joy, "/joy", self.on_joy, 1)
        self.create_timer(0.05, self.check_timeout)

    def on_joy(self, joy):
        command = Twist()
        valid = (
            len(joy.axes) > max(self.linear_axis, self.angular_axis)
            and len(joy.buttons) > self.deadman_button
        )
        if valid and joy.buttons[self.deadman_button]:
            linear = joy.axes[self.linear_axis]
            angular = joy.axes[self.angular_axis]
            if math.isfinite(linear) and math.isfinite(angular):
                command.linear.x = max(-1.0, min(1.0, linear)) * self.linear_scale
                command.angular.z = max(-1.0, min(1.0, angular)) * self.angular_scale
        self.last_joy = time.monotonic()
        self.publisher.publish(command)

    def check_timeout(self):
        if self.last_joy is None or time.monotonic() - self.last_joy > self.timeout:
            self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
