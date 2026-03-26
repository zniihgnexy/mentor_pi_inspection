#!/usr/bin/env python3
"""
Inspection manager - top-level orchestrator for the inspection workflow.
Coordinates: sensor check -> SLAM ready -> patrol start -> detect at waypoints -> record.
Provides a command interface for external control.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json
from enum import Enum


class InspectionState(Enum):
    INIT = 'init'
    SENSOR_CHECK = 'sensor_check'
    WAITING_MAP = 'waiting_map'
    READY = 'ready'
    PATROLLING = 'patrolling'
    PAUSED = 'paused'
    COMPLETED = 'completed'
    ERROR = 'error'


class InspectionManager(Node):
    """Main orchestrator for the inspection workflow."""

    def __init__(self):
        super().__init__('inspection_manager')

        self.state = InspectionState.INIT

        # Subscribers
        self.create_subscription(Bool, '/inspection/sensors_ready',
                                 self.sensors_ready_cb, 10)
        self.create_subscription(Bool, '/inspection/map_ready',
                                 self.map_ready_cb, 10)
        self.create_subscription(String, '/inspection/patrol_status',
                                 self.patrol_status_cb, 10)
        self.create_subscription(String, '/inspection/user_command',
                                 self.user_command_cb, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(String, '/inspection/command', 10)
        self.state_pub = self.create_publisher(String, '/inspection/state', 10)

        self._sensors_ok = False
        self._map_ok = False

        self.create_timer(1.0, self.state_machine_tick)
        self.get_logger().info('Inspection manager started')

    def sensors_ready_cb(self, msg: Bool):
        self._sensors_ok = msg.data

    def map_ready_cb(self, msg: Bool):
        self._map_ok = msg.data

    def patrol_status_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            if data.get('state') == 'patrolling':
                self.state = InspectionState.PATROLLING
            elif msg.data == 'patrol_complete':
                self.state = InspectionState.COMPLETED
                self.get_logger().info('Inspection patrol completed')
        except json.JSONDecodeError:
            if msg.data == 'patrol_complete':
                self.state = InspectionState.COMPLETED

    def user_command_cb(self, msg: String):
        """Handle user commands: start, stop, pause, resume."""
        cmd = msg.data.strip().lower()

        if cmd == 'start':
            if self.state in (InspectionState.READY, InspectionState.COMPLETED):
                self.start_inspection()
            else:
                self.get_logger().warn(
                    f'Cannot start in state {self.state.value}')
        elif cmd == 'stop':
            self.stop_inspection()
        elif cmd == 'pause':
            self.state = InspectionState.PAUSED
            self.send_command('stop_patrol')
        elif cmd == 'resume':
            if self.state == InspectionState.PAUSED:
                self.start_inspection()

    def state_machine_tick(self):
        """Periodic state machine update."""
        if self.state == InspectionState.INIT:
            self.state = InspectionState.SENSOR_CHECK

        elif self.state == InspectionState.SENSOR_CHECK:
            if self._sensors_ok:
                self.state = InspectionState.WAITING_MAP
                self.get_logger().info('Sensors OK, waiting for map')

        elif self.state == InspectionState.WAITING_MAP:
            if self._map_ok:
                self.state = InspectionState.READY
                self.get_logger().info('Map ready, system is READY')

        # Publish current state
        state_msg = String()
        state_msg.data = json.dumps({
            'state': self.state.value,
            'sensors_ok': self._sensors_ok,
            'map_ok': self._map_ok,
        })
        self.state_pub.publish(state_msg)

    def start_inspection(self):
        self.get_logger().info('Starting inspection patrol')
        self.state = InspectionState.PATROLLING
        self.send_command('start_patrol')

    def stop_inspection(self):
        self.get_logger().info('Stopping inspection')
        self.state = InspectionState.READY
        self.send_command('stop_patrol')

    def send_command(self, action: str, **kwargs):
        msg = String()
        cmd = {'action': action}
        cmd.update(kwargs)
        msg.data = json.dumps(cmd)
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = InspectionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
