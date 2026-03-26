#!/usr/bin/env python3
"""
Sensor manager - monitors all sensor health and provides unified status.
Acts as the top-level sensor abstraction for the control layer.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import json


class SensorManager(Node):
    """Monitors and reports overall sensor system health."""

    def __init__(self):
        super().__init__('sensor_manager')

        self.sensor_status = {
            'lidar': False,
            'camera': False,
        }

        self.create_subscription(Bool, '/inspection/lidar_status',
                                 self.lidar_status_cb, 10)
        self.create_subscription(Bool, '/inspection/camera_status',
                                 self.camera_status_cb, 10)

        self.status_pub = self.create_publisher(
            String, '/inspection/sensor_status', 10)
        self.ready_pub = self.create_publisher(
            Bool, '/inspection/sensors_ready', 10)

        self.create_timer(1.0, self.publish_status)
        self.get_logger().info('Sensor manager started')

    def lidar_status_cb(self, msg: Bool):
        self.sensor_status['lidar'] = msg.data

    def camera_status_cb(self, msg: Bool):
        self.sensor_status['camera'] = msg.data

    def publish_status(self):
        # Publish detailed status as JSON
        status_msg = String()
        status_msg.data = json.dumps(self.sensor_status)
        self.status_pub.publish(status_msg)

        # Publish overall readiness
        ready = Bool()
        ready.data = all(self.sensor_status.values())
        self.ready_pub.publish(ready)

        if not ready.data:
            offline = [k for k, v in self.sensor_status.items() if not v]
            self.get_logger().warn(f'Sensors offline: {offline}')


def main(args=None):
    rclpy.init(args=args)
    node = SensorManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
