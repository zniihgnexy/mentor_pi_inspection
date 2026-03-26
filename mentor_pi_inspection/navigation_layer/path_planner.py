#!/usr/bin/env python3
"""
Path planner for inspection routes.
Generates waypoints covering the classroom inspection area,
then uses Nav2 to navigate between them sequentially.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Bool, Int32
import json
import math
import yaml
import os


class PathPlanner(Node):
    """Generates and executes inspection patrol routes."""

    def __init__(self):
        super().__init__('path_planner')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('patrol_speed', 0.3)
        self.declare_parameter('waypoint_tolerance', 0.3)

        self.waypoints_file = self.get_parameter('waypoints_file').value
        self.patrol_speed = self.get_parameter('patrol_speed').value

        # Nav2 action clients
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        self.follow_wp_client = ActionClient(
            self, FollowWaypoints, 'follow_waypoints')

        # State
        self.waypoints = []
        self.current_wp_index = 0
        self.is_patrolling = False
        self._map = None

        # Subscribers
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(String, '/inspection/command',
                                 self.command_callback, 10)

        # Publishers
        self.status_pub = self.create_publisher(
            String, '/inspection/patrol_status', 10)
        self.wp_index_pub = self.create_publisher(
            Int32, '/inspection/current_waypoint', 10)
        self.arrived_pub = self.create_publisher(
            String, '/inspection/waypoint_arrived', 10)

        self.create_timer(1.0, self.publish_status)

        # Load waypoints from file if provided
        if self.waypoints_file and os.path.exists(self.waypoints_file):
            self.load_waypoints(self.waypoints_file)

        self.get_logger().info('Path planner initialized')

    def map_callback(self, msg: OccupancyGrid):
        self._map = msg

    def command_callback(self, msg: String):
        """Handle patrol commands: start, stop, save_waypoints, add_waypoint."""
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            cmd = {'action': msg.data}

        action = cmd.get('action', '')

        if action == 'start_patrol':
            self.start_patrol()
        elif action == 'stop_patrol':
            self.stop_patrol()
        elif action == 'add_waypoint':
            x = cmd.get('x', 0.0)
            y = cmd.get('y', 0.0)
            yaw = cmd.get('yaw', 0.0)
            self.add_waypoint(x, y, yaw)
        elif action == 'save_waypoints':
            filepath = cmd.get('file', os.path.expanduser(
                '~/inspection_maps/waypoints.yaml'))
            self.save_waypoints(filepath)
        elif action == 'generate_grid':
            self.generate_grid_waypoints(
                cmd.get('x_min', -5.0), cmd.get('x_max', 5.0),
                cmd.get('y_min', -5.0), cmd.get('y_max', 5.0),
                cmd.get('spacing', 1.5))

    def add_waypoint(self, x: float, y: float, yaw: float = 0.0):
        """Add a waypoint to the patrol route."""
        wp = {'x': x, 'y': y, 'yaw': yaw}
        self.waypoints.append(wp)
        self.get_logger().info(
            f'Waypoint added: ({x:.2f}, {y:.2f}, {yaw:.2f}), '
            f'total: {len(self.waypoints)}')

    def generate_grid_waypoints(self, x_min, x_max, y_min, y_max, spacing):
        """Generate a boustrophedon (zigzag) coverage pattern."""
        self.waypoints.clear()
        y = y_min
        forward = True
        while y <= y_max:
            if forward:
                x = x_min
                while x <= x_max:
                    self.waypoints.append({'x': x, 'y': y, 'yaw': 0.0})
                    x += spacing
            else:
                x = x_max
                while x >= x_min:
                    self.waypoints.append({'x': x, 'y': y, 'yaw': math.pi})
                    x -= spacing
            forward = not forward
            y += spacing

        self.get_logger().info(
            f'Generated {len(self.waypoints)} grid waypoints')

    def save_waypoints(self, filepath: str):
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        with open(filepath, 'w') as f:
            yaml.dump({'waypoints': self.waypoints}, f)
        self.get_logger().info(f'Waypoints saved to {filepath}')

    def load_waypoints(self, filepath: str):
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        self.waypoints = data.get('waypoints', [])
        self.get_logger().info(
            f'Loaded {len(self.waypoints)} waypoints from {filepath}')

    def start_patrol(self):
        """Start sequential waypoint patrol using Nav2."""
        if not self.waypoints:
            self.get_logger().warn('No waypoints defined, cannot start patrol')
            return
        if self.is_patrolling:
            self.get_logger().warn('Already patrolling')
            return

        self.is_patrolling = True
        self.current_wp_index = 0
        self.get_logger().info(
            f'Starting patrol with {len(self.waypoints)} waypoints')
        self.navigate_to_next()

    def stop_patrol(self):
        self.is_patrolling = False
        self.get_logger().info('Patrol stopped')

    def navigate_to_next(self):
        """Send next waypoint goal to Nav2."""
        if not self.is_patrolling:
            return
        if self.current_wp_index >= len(self.waypoints):
            self.get_logger().info('Patrol complete - all waypoints visited')
            self.is_patrolling = False
            status = String()
            status.data = 'patrol_complete'
            self.status_pub.publish(status)
            return

        wp = self.waypoints[self.current_wp_index]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp['x'])
        goal.pose.pose.position.y = float(wp['y'])
        # Convert yaw to quaternion (simplified, z-axis only)
        yaw = float(wp.get('yaw', 0.0))
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'Navigating to waypoint {self.current_wp_index}: '
            f'({wp["x"]:.2f}, {wp["y"]:.2f})')

        # Publish current index
        idx_msg = Int32()
        idx_msg.data = self.current_wp_index
        self.wp_index_pub.publish(idx_msg)

        self.nav_to_pose_client.wait_for_server()
        future = self.nav_to_pose_client.send_goal_async(
            goal, feedback_callback=self.nav_feedback_cb)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future):
        """Called when navigation to a waypoint completes."""
        wp = self.waypoints[self.current_wp_index]

        # Notify that we arrived at this waypoint
        arrived = String()
        arrived.data = json.dumps({
            'waypoint_index': self.current_wp_index,
            'x': wp['x'], 'y': wp['y']
        })
        self.arrived_pub.publish(arrived)

        self.get_logger().info(
            f'Arrived at waypoint {self.current_wp_index}')
        self.current_wp_index += 1
        self.navigate_to_next()

    def nav_feedback_cb(self, feedback_msg):
        pass

    def publish_status(self):
        status = String()
        if self.is_patrolling:
            status.data = json.dumps({
                'state': 'patrolling',
                'current_wp': self.current_wp_index,
                'total_wp': len(self.waypoints)
            })
        else:
            status.data = json.dumps({
                'state': 'idle',
                'total_wp': len(self.waypoints)
            })
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
