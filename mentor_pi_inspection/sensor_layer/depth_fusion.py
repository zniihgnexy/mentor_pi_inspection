#!/usr/bin/env python3
"""
Depth camera + Lidar fusion node.
Converts depth image to LaserScan-like data and merges with lidar scan
to fill blind spots and improve obstacle detection.

Fusion strategy:
1. Convert depth image to virtual laser scan (depthimage_to_laserscan equivalent)
2. Merge with real lidar scan: take the closer reading at each angle
3. Publish fused scan for SLAM and navigation
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import math


class DepthFusion(Node):
    """Fuses depth camera data with lidar for enhanced perception."""

    def __init__(self):
        super().__init__('depth_fusion')

        # Parameters
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('depth_topic', '/inspection/depth_image')
        self.declare_parameter('camera_info_topic', '/inspection/camera_info')
        self.declare_parameter('fused_topic', '/inspection/fused_scan')
        self.declare_parameter('depth_min_range', 0.2)
        self.declare_parameter('depth_max_range', 5.0)
        self.declare_parameter('fusion_enabled', True)
        self.declare_parameter('camera_height', 0.15)  # camera height from ground
        self.declare_parameter('scan_height_min', 0.05)  # min obstacle height
        self.declare_parameter('scan_height_max', 0.5)   # max obstacle height

        lidar_topic = self.get_parameter('lidar_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        fused_topic = self.get_parameter('fused_topic').value
        self.depth_min = self.get_parameter('depth_min_range').value
        self.depth_max = self.get_parameter('depth_max_range').value
        self.fusion_enabled = self.get_parameter('fusion_enabled').value
        self.cam_height = self.get_parameter('camera_height').value
        self.scan_h_min = self.get_parameter('scan_height_min').value
        self.scan_h_max = self.get_parameter('scan_height_max').value

        self.bridge = CvBridge()
        self._camera_info = None
        self._latest_lidar = None
        self._latest_depth_scan = None

        # Subscribers
        self.create_subscription(LaserScan, lidar_topic, self.lidar_cb, 10)
        self.create_subscription(Image, depth_topic, self.depth_cb, 10)
        self.create_subscription(CameraInfo, info_topic, self.info_cb, 10)

        # Publisher
        self.fused_pub = self.create_publisher(LaserScan, fused_topic, 10)

        self.create_timer(0.1, self.fuse_and_publish)  # 10Hz fusion
        self.get_logger().info('Depth fusion node started')

    def info_cb(self, msg: CameraInfo):
        self._camera_info = msg

    def lidar_cb(self, msg: LaserScan):
        self._latest_lidar = msg

    def depth_cb(self, msg: Image):
        """Convert depth image to virtual laser scan."""
        if self._camera_info is None:
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')
            return

        depth = depth.astype(np.float32)
        # If depth is in mm, convert to meters
        if np.nanmax(depth) > 100:
            depth = depth / 1000.0

        self._latest_depth_scan = self.depth_to_scan(depth)

    def depth_to_scan(self, depth: np.ndarray) -> LaserScan:
        """
        Convert depth image to a virtual LaserScan.
        Takes the center rows of the depth image and projects to 2D.
        """
        h, w = depth.shape[:2]
        fx = self._camera_info.k[0]  # focal length x
        cx = self._camera_info.k[2]  # principal point x
        fy = self._camera_info.k[4]  # focal length y
        cy = self._camera_info.k[5]  # principal point y

        # Use center band of image for scan
        band_top = int(h * 0.3)
        band_bot = int(h * 0.7)
        depth_band = depth[band_top:band_bot, :]

        # For each column, get the minimum valid depth (closest obstacle)
        min_depths = np.full(w, float('inf'))
        for row in range(depth_band.shape[0]):
            for col in range(w):
                d = depth_band[row, col]
                if self.depth_min < d < self.depth_max and not np.isnan(d):
                    # Check if obstacle is at valid height
                    pixel_y = band_top + row
                    obj_height = self.cam_height - d * (pixel_y - cy) / fy
                    if self.scan_h_min < obj_height < self.scan_h_max:
                        if d < min_depths[col]:
                            min_depths[col] = d

        # Convert pixel columns to angles
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'camera_link'

        # Camera FOV from focal length
        fov = 2.0 * math.atan2(w / 2.0, fx)
        scan.angle_min = -fov / 2.0
        scan.angle_max = fov / 2.0
        scan.angle_increment = fov / w
        scan.range_min = self.depth_min
        scan.range_max = self.depth_max

        # Convert: pixel column -> angle -> range
        ranges = []
        for col in range(w):
            angle = math.atan2(col - cx, fx)
            d = min_depths[col]
            if d < float('inf'):
                # Depth is along camera Z, convert to planar range
                ranges.append(d / math.cos(angle) if math.cos(angle) > 0.01 else float('inf'))
            else:
                ranges.append(float('inf'))

        # Reverse so angles go from min to max (left to right in robot frame)
        scan.ranges = list(reversed(ranges))
        return scan

    def fuse_and_publish(self):
        """Merge lidar and depth-derived scans."""
        if self._latest_lidar is None:
            return

        if not self.fusion_enabled or self._latest_depth_scan is None:
            # Pass through lidar only
            self.fused_pub.publish(self._latest_lidar)
            return

        lidar = self._latest_lidar
        depth_scan = self._latest_depth_scan

        # Create fused scan based on lidar structure
        fused = LaserScan()
        fused.header = lidar.header
        fused.angle_min = lidar.angle_min
        fused.angle_max = lidar.angle_max
        fused.angle_increment = lidar.angle_increment
        fused.time_increment = lidar.time_increment
        fused.scan_time = lidar.scan_time
        fused.range_min = lidar.range_min
        fused.range_max = lidar.range_max

        fused_ranges = list(lidar.ranges)

        # Map depth scan angles onto lidar scan indices and merge
        for i, d_range in enumerate(depth_scan.ranges):
            if d_range >= depth_scan.range_max or d_range <= depth_scan.range_min:
                continue

            d_angle = depth_scan.angle_min + i * depth_scan.angle_increment
            # Find corresponding lidar index
            if lidar.angle_min <= d_angle <= lidar.angle_max:
                idx = int((d_angle - lidar.angle_min) / lidar.angle_increment)
                if 0 <= idx < len(fused_ranges):
                    lidar_r = fused_ranges[idx]
                    # Take the closer reading (more conservative)
                    if d_range < lidar_r or lidar_r < lidar.range_min:
                        fused_ranges[idx] = d_range

        fused.ranges = fused_ranges
        self.fused_pub.publish(fused)


def main(args=None):
    rclpy.init(args=args)
    node = DepthFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
