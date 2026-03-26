#!/usr/bin/env python3
"""
Motion controller for MentorPi Mecanum wheel chassis.
Provides velocity command interface and safety limits.
Translates high-level velocity commands to chassis-compatible format.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class MotionController(Node):
    """Mecanum wheel motion controller with safety limits."""

    def __init__(self):
        super().__init__('motion_controller')

        self.declare_parameter('max_linear_x', 0.5)
        self.declare_parameter('max_linear_y', 0.5)
        self.declare_parameter('max_angular_z', 1.0)
        self.declare_parameter('cmd_input_topic', '/inspection/cmd_vel')
        self.declare_parameter('cmd_output_topic', '/cmd_vel')
        self.declare_parameter('enable_safety', True)

        self.max_vx = self.get_parameter('max_linear_x').value
        self.max_vy = self.get_parameter('max_linear_y').value
        self.max_wz = self.get_parameter('max_angular_z').value
        self.safety_enabled = self.get_parameter('enable_safety').value

        cmd_in = self.get_parameter('cmd_input_topic').value
        cmd_out = self.get_parameter('cmd_output_topic').value

        self.cmd_sub = self.create_subscription(Twist, cmd_in, self.cmd_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_out, 10)

        self.create_subscription(Bool, '/inspection/emergency_stop',
                                 self.estop_callback, 10)

        self._estop = False
        self.get_logger().info('Motion controller started (Mecanum)')

    def cmd_callback(self, msg: Twist):
        if self._estop:
            self.stop()
            return

        safe_cmd = Twist()
        safe_cmd.linear.x = self.clamp(msg.linear.x, -self.max_vx, self.max_vx)
        safe_cmd.linear.y = self.clamp(msg.linear.y, -self.max_vy, self.max_vy)
        safe_cmd.angular.z = self.clamp(msg.angular.z, -self.max_wz, self.max_wz)
        self.cmd_pub.publish(safe_cmd)

    def estop_callback(self, msg: Bool):
        self._estop = msg.data
        if self._estop:
            self.get_logger().warn('Emergency stop activated')
            self.stop()

    def stop(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def clamp(value, min_val, max_val):
        return max(min_val, min(value, max_val))


def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
