#!/usr/bin/env python3
"""Minimal keyboard teleop for SLAM mapping."""
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool


HELP_TEXT = """
Keyboard teleop
  w/s : forward/backward
  a/d : strafe left/right
  q/e : rotate left/right
  space or k : emergency stop
  Ctrl-C : quit
"""


class KeyboardTeleop(Node):
    """Keyboard teleop for manual MentorPi movement."""

    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('cmd_vel_topic', '/inspection/cmd_vel')
        self.declare_parameter('emergency_stop_topic', '/inspection/emergency_stop')
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('lateral_speed', 0.25)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('idle_timeout', 0.25)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.estop_topic = self.get_parameter('emergency_stop_topic').value
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.lateral_speed = float(self.get_parameter('lateral_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.idle_timeout = float(self.get_parameter('idle_timeout').value)

        if not sys.stdin.isatty():
            raise RuntimeError('keyboard_teleop requires an interactive terminal')

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.estop_pub = self.create_publisher(Bool, self.estop_topic, 10)

        self._fd = sys.stdin.fileno()
        self._settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)

        self._moving = False
        self._estop = False
        self._last_input_time = self.get_clock().now()

        self.create_timer(0.05, self.poll_keyboard)
        for line in HELP_TEXT.strip().splitlines():
            self.get_logger().info(line)

    def poll_keyboard(self):
        key = self.read_key()
        now = self.get_clock().now()

        if key:
            self._last_input_time = now
            if key == '\x03':
                self.stop()
                rclpy.shutdown()
                return
            self.handle_key(key)
        elif self._moving:
            elapsed = (now - self._last_input_time).nanoseconds / 1e9
            if elapsed >= self.idle_timeout:
                self.stop()

    def read_key(self):
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return ''
        return sys.stdin.read(1)

    def handle_key(self, key: str):
        key = key.lower()

        if key in (' ', 'k'):
            self.publish_estop(True)
            self.stop()
            self._estop = True
            return

        twist = Twist()
        if key == 'w':
            twist.linear.x = self.linear_speed
        elif key == 's':
            twist.linear.x = -self.linear_speed
        elif key == 'a':
            twist.linear.y = self.lateral_speed
        elif key == 'd':
            twist.linear.y = -self.lateral_speed
        elif key == 'q':
            twist.angular.z = self.angular_speed
        elif key == 'e':
            twist.angular.z = -self.angular_speed
        else:
            return

        if self._estop:
            self.publish_estop(False)
            self._estop = False

        self.cmd_pub.publish(twist)
        self._moving = True

    def stop(self):
        self.cmd_pub.publish(Twist())
        self._moving = False

    def publish_estop(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        self.estop_pub.publish(msg)

    def restore_terminal(self):
        if self._settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._settings)
            self._settings = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardTeleop()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.stop()
            node.restore_terminal()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
