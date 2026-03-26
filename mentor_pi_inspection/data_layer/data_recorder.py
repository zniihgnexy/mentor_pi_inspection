#!/usr/bin/env python3
"""
Data recorder - logs inspection results with timestamps and positions.
Supports CSV, JSON, and SQLite output formats.
Each inspection record: timestamp, position (x,y), power_status, confidence, details.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
import json
import csv
import sqlite3
import os
from datetime import datetime


class DataRecorder(Node):
    """Records inspection data to CSV/JSON/SQLite."""

    def __init__(self):
        super().__init__('data_recorder')

        self.declare_parameter('output_dir', os.path.expanduser('~/inspection_data'))
        self.declare_parameter('output_format', 'csv')  # csv, json, sqlite
        self.declare_parameter('session_name', '')

        self.output_dir = self.get_parameter('output_dir').value
        self.output_format = self.get_parameter('output_format').value
        session = self.get_parameter('session_name').value
        if not session:
            session = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.session_name = session

        os.makedirs(self.output_dir, exist_ok=True)

        # Current robot pose
        self.current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.records = []

        # Subscribers
        self.create_subscription(
            String, '/inspection/power_status', self.power_status_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, 10)
        self.create_subscription(
            String, '/inspection/waypoint_arrived', self.waypoint_cb, 10)

        # Publisher for record confirmation
        self.record_pub = self.create_publisher(
            String, '/inspection/record_saved', 10)

        # Initialize storage
        self._init_storage()

        self.get_logger().info(
            f'Data recorder started: format={self.output_format}, '
            f'session={self.session_name}')

    def _init_storage(self):
        """Initialize the output storage backend."""
        base = os.path.join(self.output_dir, self.session_name)

        if self.output_format == 'csv':
            self.csv_path = base + '.csv'
            with open(self.csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 'x', 'y', 'yaw', 'waypoint_index',
                    'power_status', 'confidence', 'led_count', 'reason'
                ])
            self.get_logger().info(f'CSV file: {self.csv_path}')

        elif self.output_format == 'json':
            self.json_path = base + '.json'
            self.get_logger().info(f'JSON file: {self.json_path}')

        elif self.output_format == 'sqlite':
            self.db_path = base + '.db'
            conn = sqlite3.connect(self.db_path)
            conn.execute('''CREATE TABLE IF NOT EXISTS inspection_records (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT, x REAL, y REAL, yaw REAL,
                waypoint_index INTEGER, power_status TEXT,
                confidence REAL, led_count INTEGER, reason TEXT
            )''')
            conn.commit()
            conn.close()
            self.get_logger().info(f'SQLite DB: {self.db_path}')

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        """Update current robot position."""
        p = msg.pose.pose
        self.current_pose['x'] = p.position.x
        self.current_pose['y'] = p.position.y
        # Extract yaw from quaternion
        q = p.orientation
        import math
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_pose['yaw'] = math.atan2(siny, cosy)

    def waypoint_cb(self, msg: String):
        """Track current waypoint for record association."""
        try:
            data = json.loads(msg.data)
            self._current_wp_index = data.get('waypoint_index', -1)
        except json.JSONDecodeError:
            pass

    _current_wp_index = -1

    def power_status_cb(self, msg: String):
        """Record a power status detection result."""
        try:
            detection = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        record = {
            'timestamp': datetime.now().isoformat(),
            'x': round(self.current_pose['x'], 3),
            'y': round(self.current_pose['y'], 3),
            'yaw': round(self.current_pose['yaw'], 3),
            'waypoint_index': self._current_wp_index,
            'power_status': detection.get('status', 'unknown'),
            'confidence': detection.get('confidence', 0.0),
            'led_count': detection.get('led_count', 0),
            'reason': detection.get('reason', ''),
        }

        self.records.append(record)
        self._save_record(record)

        confirm = String()
        confirm.data = json.dumps(record)
        self.record_pub.publish(confirm)

        self.get_logger().info(
            f'Record #{len(self.records)}: pos=({record["x"]}, {record["y"]}), '
            f'status={record["power_status"]}, conf={record["confidence"]}')

    def _save_record(self, record: dict):
        """Persist record to the configured storage backend."""
        if self.output_format == 'csv':
            with open(self.csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    record['timestamp'], record['x'], record['y'],
                    record['yaw'], record['waypoint_index'],
                    record['power_status'], record['confidence'],
                    record['led_count'], record['reason']
                ])

        elif self.output_format == 'json':
            with open(self.json_path, 'w') as f:
                json.dump(self.records, f, indent=2, ensure_ascii=False)

        elif self.output_format == 'sqlite':
            conn = sqlite3.connect(self.db_path)
            conn.execute(
                '''INSERT INTO inspection_records
                   (timestamp, x, y, yaw, waypoint_index,
                    power_status, confidence, led_count, reason)
                   VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)''',
                (record['timestamp'], record['x'], record['y'],
                 record['yaw'], record['waypoint_index'],
                 record['power_status'], record['confidence'],
                 record['led_count'], record['reason']))
            conn.commit()
            conn.close()

    def generate_summary(self) -> dict:
        """Generate inspection session summary."""
        if not self.records:
            return {'total': 0}

        total = len(self.records)
        on_count = sum(1 for r in self.records if r['power_status'] == 'on')
        off_count = sum(1 for r in self.records if r['power_status'] == 'off')
        uncertain_count = total - on_count - off_count

        return {
            'session': self.session_name,
            'total_records': total,
            'power_on': on_count,
            'power_off': off_count,
            'uncertain': uncertain_count,
            'on_ratio': round(on_count / total, 2) if total > 0 else 0,
        }


def main(args=None):
    rclpy.init(args=args)
    node = DataRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
