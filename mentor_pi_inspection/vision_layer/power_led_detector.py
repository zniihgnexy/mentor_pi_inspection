#!/usr/bin/env python3
"""
Power LED detector - identifies computer power indicator light status.
Uses HSV color analysis + contour detection (no deep learning).
Method is explainable and evaluable as required.

Detection logic:
1. Convert ROI to HSV color space
2. Apply color masks for common LED colors (green=on, off=dark, amber=uncertain)
3. Analyze brightness and color distribution
4. Output: 'on' / 'off' / 'uncertain'
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json


class PowerLedDetector(Node):
    """Detects computer power LED status from camera images."""

    def __init__(self):
        super().__init__('power_led_detector')

        # Parameters
        self.declare_parameter('image_topic', '/inspection/rgb_image')
        self.declare_parameter('detection_enabled', True)
        self.declare_parameter('roi_x_ratio', 0.3)   # ROI center x as ratio of image width
        self.declare_parameter('roi_y_ratio', 0.6)   # ROI center y as ratio of image height
        self.declare_parameter('roi_w_ratio', 0.4)   # ROI width ratio
        self.declare_parameter('roi_h_ratio', 0.3)   # ROI height ratio
        self.declare_parameter('brightness_threshold', 80)
        self.declare_parameter('led_min_area', 30)
        self.declare_parameter('led_max_area', 5000)
        self.declare_parameter('debug_image', True)

        image_topic = self.get_parameter('image_topic').value
        self.enabled = self.get_parameter('detection_enabled').value
        self.roi_x_ratio = self.get_parameter('roi_x_ratio').value
        self.roi_y_ratio = self.get_parameter('roi_y_ratio').value
        self.roi_w_ratio = self.get_parameter('roi_w_ratio').value
        self.roi_h_ratio = self.get_parameter('roi_h_ratio').value
        self.brightness_thresh = self.get_parameter('brightness_threshold').value
        self.led_min_area = self.get_parameter('led_min_area').value
        self.led_max_area = self.get_parameter('led_max_area').value
        self.debug_image_enabled = self.get_parameter('debug_image').value

        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)

        # Publishers
        self.result_pub = self.create_publisher(
            String, '/inspection/power_status', 10)
        self.debug_pub = self.create_publisher(
            Image, '/inspection/debug_image', 10)

        # LED color ranges in HSV
        # Green LED (power on indicator)
        self.green_lower = np.array([35, 80, 80])
        self.green_upper = np.array([85, 255, 255])
        # Blue LED (some machines use blue for power on)
        self.blue_lower = np.array([90, 80, 80])
        self.blue_upper = np.array([130, 255, 255])
        # White/bright LED
        self.white_lower = np.array([0, 0, 200])
        self.white_upper = np.array([180, 40, 255])
        # Amber/orange LED (sleep/uncertain)
        self.amber_lower = np.array([10, 80, 80])
        self.amber_upper = np.array([25, 255, 255])

        self.get_logger().info('Power LED detector initialized')

    def image_callback(self, msg: Image):
        if not self.enabled:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        result = self.detect_power_led(frame)
        self.publish_result(result)

    def detect_power_led(self, frame: np.ndarray) -> dict:
        """
        Main detection pipeline:
        1. Extract ROI from frame
        2. Convert to HSV
        3. Apply color masks for LED colors
        4. Find bright contours
        5. Classify based on color and brightness
        """
        h, w = frame.shape[:2]

        # Calculate ROI coordinates
        roi_x = int(w * self.roi_x_ratio - w * self.roi_w_ratio / 2)
        roi_y = int(h * self.roi_y_ratio - h * self.roi_h_ratio / 2)
        roi_w = int(w * self.roi_w_ratio)
        roi_h = int(h * self.roi_h_ratio)

        # Clamp to image bounds
        roi_x = max(0, roi_x)
        roi_y = max(0, roi_y)
        roi_w = min(roi_w, w - roi_x)
        roi_h = min(roi_h, h - roi_y)

        roi = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
        if roi.size == 0:
            return {'status': 'uncertain', 'confidence': 0.0, 'reason': 'empty_roi'}

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Detect bright spots (potential LEDs)
        _, bright_mask = cv2.threshold(gray, self.brightness_thresh, 255, cv2.THRESH_BINARY)

        # Color masks
        green_mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
        blue_mask = cv2.inRange(hsv, self.blue_lower, self.blue_upper)
        white_mask = cv2.inRange(hsv, self.white_lower, self.white_upper)
        amber_mask = cv2.inRange(hsv, self.amber_lower, self.amber_upper)

        # Combine ON indicators (green, blue, white) with brightness
        on_mask = cv2.bitwise_and(
            bright_mask,
            cv2.bitwise_or(green_mask, cv2.bitwise_or(blue_mask, white_mask))
        )
        # Amber with brightness = uncertain/sleep
        uncertain_mask = cv2.bitwise_and(bright_mask, amber_mask)

        # Find contours
        on_contours = self.find_led_contours(on_mask)
        uncertain_contours = self.find_led_contours(uncertain_mask)

        # Classification logic
        result = self.classify(on_contours, uncertain_contours, bright_mask)

        # Debug visualization
        if self.debug_image_enabled:
            self.publish_debug(frame, roi_x, roi_y, roi_w, roi_h,
                               on_contours, uncertain_contours, result)

        return result

    def find_led_contours(self, mask: np.ndarray) -> list:
        """Find contours that match LED size criteria."""
        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        valid = []
        for c in contours:
            area = cv2.contourArea(c)
            if self.led_min_area <= area <= self.led_max_area:
                # Check circularity (LEDs tend to be round)
                perimeter = cv2.arcLength(c, True)
                if perimeter > 0:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    if circularity > 0.3:  # Reasonably circular
                        valid.append(c)
        return valid

    def classify(self, on_contours, uncertain_contours, bright_mask) -> dict:
        """Classify power status based on detected LED contours."""
        on_count = len(on_contours)
        uncertain_count = len(uncertain_contours)
        bright_ratio = np.count_nonzero(bright_mask) / max(bright_mask.size, 1)

        if on_count > 0:
            max_area = max(cv2.contourArea(c) for c in on_contours)
            confidence = min(1.0, max_area / self.led_max_area + 0.3)
            return {
                'status': 'on',
                'confidence': round(confidence, 2),
                'led_count': on_count,
                'reason': f'detected {on_count} active LED(s)'
            }
        elif uncertain_count > 0:
            return {
                'status': 'uncertain',
                'confidence': 0.5,
                'led_count': uncertain_count,
                'reason': f'detected {uncertain_count} amber LED(s), possible sleep mode'
            }
        elif bright_ratio < 0.01:
            return {
                'status': 'off',
                'confidence': 0.8,
                'led_count': 0,
                'reason': 'no bright spots detected in ROI'
            }
        else:
            return {
                'status': 'uncertain',
                'confidence': 0.3,
                'led_count': 0,
                'reason': 'ambient light detected but no clear LED pattern'
            }

    def publish_result(self, result: dict):
        msg = String()
        msg.data = json.dumps(result)
        self.result_pub.publish(msg)

    def publish_debug(self, frame, rx, ry, rw, rh,
                      on_contours, uncertain_contours, result):
        """Publish annotated debug image."""
        debug = frame.copy()
        # Draw ROI
        cv2.rectangle(debug, (rx, ry), (rx+rw, ry+rh), (255, 255, 0), 2)

        # Draw detected LEDs
        for c in on_contours:
            c_shifted = c.copy()
            c_shifted[:, :, 0] += rx
            c_shifted[:, :, 1] += ry
            cv2.drawContours(debug, [c_shifted], -1, (0, 255, 0), 2)

        for c in uncertain_contours:
            c_shifted = c.copy()
            c_shifted[:, :, 0] += rx
            c_shifted[:, :, 1] += ry
            cv2.drawContours(debug, [c_shifted], -1, (0, 165, 255), 2)

        # Status text
        status = result['status']
        color = {'on': (0, 255, 0), 'off': (0, 0, 255),
                 'uncertain': (0, 165, 255)}[status]
        cv2.putText(debug, f"Power: {status} ({result['confidence']:.0%})",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        try:
            msg = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
            self.debug_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Debug image publish error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PowerLedDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
