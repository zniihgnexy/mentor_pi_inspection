#!/usr/bin/env python3
"""
SLAM management node - wraps slam_toolbox for map building and localization.
Supports two modes:
  1. Mapping mode: build and save a new map
  2. Localization mode: load existing map and localize
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav_msgs.msg import OccupancyGrid
from slam_toolbox.srv import SaveMap, SerializePoseGraph
from std_srvs.srv import Empty
import os


class SlamNode(Node):
    """SLAM management for mapping and localization."""

    def __init__(self):
        super().__init__('slam_node')

        self.declare_parameter('mode', 'mapping')  # 'mapping' or 'localization'
        self.declare_parameter('map_save_dir', os.path.expanduser('~/inspection_maps'))
        self.declare_parameter('map_name', 'classroom_map')

        self.mode = self.get_parameter('mode').value
        self.map_dir = self.get_parameter('map_save_dir').value
        self.map_name = self.get_parameter('map_name').value

        os.makedirs(self.map_dir, exist_ok=True)

        # Subscribe to map updates
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Publishers
        self.status_pub = self.create_publisher(String, '/inspection/slam_status', 10)
        self.map_ready_pub = self.create_publisher(Bool, '/inspection/map_ready', 10)

        # Service clients for slam_toolbox
        self.save_map_client = self.create_client(
            SerializePoseGraph, '/slam_toolbox/serialize_map')

        self._map_received = False
        self.create_timer(2.0, self.publish_status)

        self.get_logger().info(f'SLAM node started in {self.mode} mode')

    def map_callback(self, msg: OccupancyGrid):
        """Track map availability."""
        if not self._map_received:
            self.get_logger().info(
                f'Map received: {msg.info.width}x{msg.info.height}, '
                f'resolution={msg.info.resolution}m/pixel')
            self._map_received = True

    def publish_status(self):
        status = String()
        status.data = f'{self.mode}:{"ready" if self._map_received else "waiting"}'
        self.status_pub.publish(status)

        ready = Bool()
        ready.data = self._map_received
        self.map_ready_pub.publish(ready)

    def save_map(self):
        """Save current map to disk via slam_toolbox service."""
        if not self.save_map_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('slam_toolbox serialize service not available')
            return False

        request = SerializePoseGraph.Request()
        request.filename = os.path.join(self.map_dir, self.map_name)
        future = self.save_map_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None:
            self.get_logger().info(f'Map saved to {request.filename}')
            return True
        else:
            self.get_logger().error('Failed to save map')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = SlamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
