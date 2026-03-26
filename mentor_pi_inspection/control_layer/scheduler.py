#!/usr/bin/env python3
"""
Scheduler - coordinates detection timing with navigation.
When robot arrives at a waypoint, triggers a detection window
before moving to the next waypoint.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json


class Scheduler(Node):
    """Coordinates detection at each waypoint during patrol."""

    def __init__(self):
        super().__init__('scheduler')

        self.declare_parameter('detection_duration', 3.0)  # seconds to observe at each stop
        self.declare_parameter('detection_samples', 5)  # frames to analyze per stop

        self.detection_duration = self.get_parameter('detection_duration').value
        self.samples_needed = self.get_parameter('detection_samples').value

        self._detecting = False
        self._samples_collected = 0
        self._detection_timer = None

        # Subscribe to waypoint arrival
        self.create_subscription(
            String, '/inspection/waypoint_arrived', self.waypoint_arrived_cb, 10)
        # Subscribe to detection results
        self.create_subscription(
            String, '/inspection/power_status', self.detection_result_cb, 10)

        # Publishers
        self.detect_enable_pub = self.create_publisher(
            Bool, '/inspection/detect_enable', 10)
        self.status_pub = self.create_publisher(
            String, '/inspection/scheduler_status', 10)

        self.get_logger().info('Scheduler started')

    def waypoint_arrived_cb(self, msg: String):
        """Start detection window when arriving at a waypoint."""
        self.get_logger().info('Waypoint reached, starting detection window')
        self._detecting = True
        self._samples_collected = 0

        # Enable detection
        enable = Bool()
        enable.data = True
        self.detect_enable_pub.publish(enable)

        # Set timeout for detection window
        self._detection_timer = self.create_timer(
            self.detection_duration, self.end_detection_window)

    def detection_result_cb(self, msg: String):
        if self._detecting:
            self._samples_collected += 1

    def end_detection_window(self):
        """End the detection window at current waypoint."""
        self._detecting = False
        if self._detection_timer:
            self._detection_timer.cancel()
            self._detection_timer = None

        # Disable detection during transit
        enable = Bool()
        enable.data = False
        self.detect_enable_pub.publish(enable)

        self.get_logger().info(
            f'Detection window closed, collected {self._samples_collected} samples')

        status = String()
        status.data = json.dumps({
            'event': 'detection_complete',
            'samples': self._samples_collected
        })
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = Scheduler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
