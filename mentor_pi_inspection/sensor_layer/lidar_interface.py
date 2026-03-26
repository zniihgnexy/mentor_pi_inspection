#!/usr/bin/env python3
"""
Lidar sensor interface - abstracts laser scanner hardware access.
Subscribes to raw lidar data and republishes with unified topic/frame.
Ensures hardware-agnostic access for SLAM and navigation.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class LidarInterface(Node):
    """Unified lidar interface for MentorPi laser scanner."""

    def __init__(self):
        super().__init__('lidar_interface')

        # Parameters - no hardcoded device paths
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/inspection/scan')
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 12.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value

        self.scan_sub = self.create_subscription(
            LaserScan, input_topic, self.scan_callback, 10)
        self.scan_pub = self.create_publisher(
            LaserScan, output_topic, 10)
        self.status_pub = self.create_publisher(
            Bool, '/inspection/lidar_status', 10)

        self._last_scan_time = None
        self.create_timer(1.0, self.check_health)
        self.get_logger().info(f'Lidar interface started: {input_topic} -> {output_topic}')

    def scan_callback(self, msg: LaserScan):
        """Filter and republish scan data with unified frame."""
        self._last_scan_time = self.get_clock().now()

        filtered = LaserScan()
        filtered.header = msg.header
        # Keep the original frame_id so it matches the robot's TF tree
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = self.range_min
        filtered.range_max = self.range_max

        # Filter out-of-range readings
        filtered.ranges = [
            r if self.range_min <= r <= self.range_max else float('inf')
            for r in msg.ranges
        ]
        filtered.intensities = list(msg.intensities) if msg.intensities else []

        self.scan_pub.publish(filtered)

    def check_health(self):
        """Publish lidar health status."""
        status = Bool()
        if self._last_scan_time is None:
            status.data = False
        else:
            elapsed = (self.get_clock().now() - self._last_scan_time).nanoseconds / 1e9
            status.data = elapsed < 2.0
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = LidarInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
