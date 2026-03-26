#!/usr/bin/env python3
"""Keyboard teleop for manual SLAM mapping on the MentorPi mecanum chassis."""
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from mentor_pi_inspection.control_layer.keyboard_bindings import (
    HELP_KEYS,
    QUIT_KEYS,
    command_for_key,
)


HELP_TEXT = """
Keyboard teleop for SLAM mapping
  w/s : forward/backward
  a/d : strafe left/right
  q/e : rotate left/right
  space or k : emergency stop
  h or ? : show help
  Ctrl-C : quit
"""


class KeyboardTeleop(Node):
    """Publish manual velocity commands from a terminal keyboard."""

    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('cmd_vel_topic', '/inspection/cmd_vel')
        self.declare_parameter('emergency_stop_topic', '/inspection/emergency_stop')
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('lateral_speed', 0.25)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('idle_timeout', 0.25)
        self.declare_parameter('poll_period', 0.05)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.emergency_stop_topic = self.get_parameter('emergency_stop_topic').value
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.lateral_speed = float(self.get_parameter('lateral_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.idle_timeout = float(self.get_parameter('idle_timeout').value)
        self.poll_period = float(self.get_parameter('poll_period').value)

        if not sys.stdin.isatty():
            raise RuntimeError('keyboard_teleop requires an interactive terminal')

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.estop_pub = self.create_publisher(Bool, self.emergency_stop_topic, 10)

        self._terminal_fd = sys.stdin.fileno()
        self._terminal_settings = termios.tcgetattr(self._terminal_fd)
        tty.setcbreak(self._terminal_fd)

        self._motion_active = False
        self._estop_active = False
        self._last_input_time = self.get_clock().now()

        self.create_timer(self.poll_period, self.poll_keyboard)
        self.get_logger().info(
            f'Keyboard teleop publishing to {self.cmd_vel_topic} '
            f'with estop topic {self.emergency_stop_topic}')
        self.print_help()

    def poll_keyboard(self):
        """Poll stdin without blocking the ROS event loop."""
        key = self.read_key()
        now = self.get_clock().now()

        if key:
            self._last_input_time = now
            if key in QUIT_KEYS:
                self.get_logger().info('Keyboard teleop exiting')
                self.publish_stop()
                rclpy.shutdown()
                return
            if key.lower() in HELP_KEYS:
                self.print_help()
                return

            command = command_for_key(
                key,
                linear_speed=self.linear_speed,
                lateral_speed=self.lateral_speed,
                angular_speed=self.angular_speed,
            )
            if command is None:
                return

            if command.emergency_stop:
                self.publish_emergency_stop()
            else:
                self.publish_motion(command.linear_x, command.linear_y, command.angular_z)
        elif self._motion_active:
            elapsed = (now - self._last_input_time).nanoseconds / 1e9
            if elapsed >= self.idle_timeout:
                self.publish_stop()

    def read_key(self):
        """Return a single keypress if one is available."""
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return ''
        return sys.stdin.read(1)

    def publish_motion(self, linear_x: float, linear_y: float, angular_z: float):
        """Publish a movement command and release estop if needed."""
        if self._estop_active:
            self.publish_estop(False)
            self._estop_active = False

        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.linear.y = linear_y
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)
        self._motion_active = True

    def publish_stop(self):
        """Publish a zero-velocity command."""
        self.cmd_pub.publish(Twist())
        self._motion_active = False

    def publish_emergency_stop(self):
        """Latch estop and send an immediate zero command."""
        self.publish_stop()
        self.publish_estop(True)
        self._estop_active = True

    def publish_estop(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        self.estop_pub.publish(msg)

    def print_help(self):
        for line in HELP_TEXT.strip().splitlines():
            self.get_logger().info(line)

    def restore_terminal(self):
        if self._terminal_settings is not None:
            termios.tcsetattr(
                self._terminal_fd,
                termios.TCSADRAIN,
                self._terminal_settings,
            )
            self._terminal_settings = None


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
                node.publish_stop()
            node.restore_terminal()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
