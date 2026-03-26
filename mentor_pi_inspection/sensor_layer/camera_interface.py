#!/usr/bin/env python3
"""
Camera sensor interface - abstracts depth camera hardware access.
Provides unified access to RGB and depth image streams.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Bool


class CameraInterface(Node):
    """Unified camera interface for MentorPi depth camera."""

    def __init__(self):
        super().__init__('camera_interface')

        # Parameters - configurable topics for hardware portability
        self.declare_parameter('rgb_input_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_input_topic', '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('rgb_output_topic', '/inspection/rgb_image')
        self.declare_parameter('depth_output_topic', '/inspection/depth_image')
        self.declare_parameter('camera_info_output_topic', '/inspection/camera_info')

        rgb_in = self.get_parameter('rgb_input_topic').value
        depth_in = self.get_parameter('depth_input_topic').value
        info_in = self.get_parameter('camera_info_topic').value
        rgb_out = self.get_parameter('rgb_output_topic').value
        depth_out = self.get_parameter('depth_output_topic').value
        info_out = self.get_parameter('camera_info_output_topic').value

        # Subscribers (use sensor data QoS to match camera driver's best_effort publishing)
        self.rgb_sub = self.create_subscription(Image, rgb_in, self.rgb_callback, qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(Image, depth_in, self.depth_callback, qos_profile_sensor_data)
        self.info_sub = self.create_subscription(CameraInfo, info_in, self.info_callback, qos_profile_sensor_data)

        # Publishers
        self.rgb_pub = self.create_publisher(Image, rgb_out, 10)
        self.depth_pub = self.create_publisher(Image, depth_out, 10)
        self.info_pub = self.create_publisher(CameraInfo, info_out, 10)
        self.status_pub = self.create_publisher(Bool, '/inspection/camera_status', 10)

        self._last_rgb_time = None
        self._last_depth_time = None
        self.create_timer(1.0, self.check_health)
        self.get_logger().info('Camera interface started')

    def rgb_callback(self, msg: Image):
        self._last_rgb_time = self.get_clock().now()
        self.rgb_pub.publish(msg)

    def depth_callback(self, msg: Image):
        self._last_depth_time = self.get_clock().now()
        self.depth_pub.publish(msg)

    def info_callback(self, msg: CameraInfo):
        self.info_pub.publish(msg)

    def check_health(self):
        """Check both RGB and depth streams are active."""
        status = Bool()
        now = self.get_clock().now()
        rgb_ok = (self._last_rgb_time is not None and
                  (now - self._last_rgb_time).nanoseconds / 1e9 < 2.0)
        depth_ok = (self._last_depth_time is not None and
                    (now - self._last_depth_time).nanoseconds / 1e9 < 2.0)
        status.data = rgb_ok and depth_ok
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
