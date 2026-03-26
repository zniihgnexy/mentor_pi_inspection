#!/usr/bin/env python3
"""
Image processor - preprocessing utilities for the vision pipeline.
Handles image enhancement, noise reduction, and ROI extraction.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageProcessor(Node):
    """Preprocesses camera images for downstream detection."""

    def __init__(self):
        super().__init__('image_processor')

        self.declare_parameter('input_topic', '/inspection/rgb_image')
        self.declare_parameter('output_topic', '/inspection/processed_image')
        self.declare_parameter('denoise', True)
        self.declare_parameter('enhance_contrast', True)
        self.declare_parameter('target_width', 640)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.denoise = self.get_parameter('denoise').value
        self.enhance = self.get_parameter('enhance_contrast').value
        self.target_w = self.get_parameter('target_width').value

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, input_topic, self.callback, 10)
        self.pub = self.create_publisher(Image, output_topic, 10)
        self.get_logger().info('Image processor started')

    def callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        processed = self.process(frame)
        try:
            out_msg = self.bridge.cv2_to_imgmsg(processed, 'bgr8')
            out_msg.header = msg.header
            self.pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f'Publish error: {e}')

    def process(self, frame: np.ndarray) -> np.ndarray:
        """Apply preprocessing pipeline."""
        h, w = frame.shape[:2]

        # Resize if needed
        if w > self.target_w:
            scale = self.target_w / w
            frame = cv2.resize(frame, None, fx=scale, fy=scale)

        # Denoise
        if self.denoise:
            frame = cv2.fastNlMeansDenoisingColored(frame, None, 6, 6, 7, 21)

        # Enhance contrast using CLAHE
        if self.enhance:
            lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            lab[:, :, 0] = clahe.apply(lab[:, :, 0])
            frame = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        return frame


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
