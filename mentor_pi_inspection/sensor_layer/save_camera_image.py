#!/usr/bin/env python3
"""Save a single camera frame to disk and exit."""
import os
import sys
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class SaveCameraImage(Node):
    def __init__(self):
        super().__init__('save_camera_image')

        self.declare_parameter('topic', '/ascamera/camera_publisher/rgb0/image')
        self.declare_parameter('output_dir', os.path.expanduser('~/inspection_data/camera_raw'))

        topic = self.get_parameter('topic').value
        self.output_dir = self.get_parameter('output_dir').value
        self.bridge = CvBridge()
        self.saved = False

        os.makedirs(self.output_dir, exist_ok=True)

        self.create_subscription(Image, topic, self.image_callback, qos_profile_sensor_data)
        self.get_logger().info(f'Waiting for image on {topic} ...')

    def image_callback(self, msg: Image):
        if self.saved:
            return
        self.saved = True

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filepath = os.path.join(self.output_dir, f'camera_{stamp}.jpg')
        cv2.imwrite(filepath, cv_image)
        self.get_logger().info(f'Saved: {filepath}')

        raise SystemExit(0)


def main(args=None):
    rclpy.init(args=args)
    node = SaveCameraImage()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
