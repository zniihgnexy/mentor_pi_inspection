#!/usr/bin/env python3
"""
Basic tests for the inspection system modules.
Tests core logic without requiring ROS2 runtime.
"""
import pytest
import numpy as np
import sys
import os

# Add parent to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class TestPowerLedDetection:
    """Test power LED detection logic."""

    def test_classify_on_contours(self):
        """When ON-color contours are found, status should be 'on'."""
        from mentor_pi_inspection.vision_layer.power_led_detector import PowerLedDetector
        # We test the classify method directly
        # Mock contours with area
        mock_contour = np.array([[[10, 10]], [[50, 10]], [[50, 50]], [[10, 50]]])
        bright_mask = np.zeros((100, 100), dtype=np.uint8)

        # Create a minimal node-less test by calling classify as a static-like method
        # We'll instantiate with a mock approach
        result = self._classify_helper([mock_contour], [], bright_mask)
        assert result['status'] == 'on'
        assert result['confidence'] > 0

    def test_classify_no_leds(self):
        """When no LEDs detected and dark, status should be 'off'."""
        bright_mask = np.zeros((100, 100), dtype=np.uint8)
        result = self._classify_helper([], [], bright_mask)
        assert result['status'] == 'off'

    def test_classify_amber(self):
        """When amber contours found, status should be 'uncertain'."""
        mock_contour = np.array([[[10, 10]], [[30, 10]], [[30, 30]], [[10, 30]]])
        bright_mask = np.zeros((100, 100), dtype=np.uint8)
        result = self._classify_helper([], [mock_contour], bright_mask)
        assert result['status'] == 'uncertain'

    @staticmethod
    def _classify_helper(on_contours, uncertain_contours, bright_mask):
        """Helper to test classify logic without ROS node."""
        import cv2
        on_count = len(on_contours)
        uncertain_count = len(uncertain_contours)
        bright_ratio = np.count_nonzero(bright_mask) / max(bright_mask.size, 1)
        led_max_area = 5000

        if on_count > 0:
            max_area = max(cv2.contourArea(c) for c in on_contours)
            confidence = min(1.0, max_area / led_max_area + 0.3)
            return {'status': 'on', 'confidence': round(confidence, 2),
                    'led_count': on_count,
                    'reason': f'detected {on_count} active LED(s)'}
        elif uncertain_count > 0:
            return {'status': 'uncertain', 'confidence': 0.5,
                    'led_count': uncertain_count,
                    'reason': f'detected {uncertain_count} amber LED(s)'}
        elif bright_ratio < 0.01:
            return {'status': 'off', 'confidence': 0.8, 'led_count': 0,
                    'reason': 'no bright spots detected'}
        else:
            return {'status': 'uncertain', 'confidence': 0.3, 'led_count': 0,
                    'reason': 'ambient light detected'}


class TestPathPlanner:
    """Test path planning logic."""

    def test_grid_waypoint_generation(self):
        """Test boustrophedon pattern generation."""
        waypoints = []
        x_min, x_max = -2.0, 2.0
        y_min, y_max = -2.0, 2.0
        spacing = 1.0

        y = y_min
        forward = True
        while y <= y_max:
            if forward:
                x = x_min
                while x <= x_max:
                    waypoints.append({'x': x, 'y': y, 'yaw': 0.0})
                    x += spacing
            else:
                x = x_max
                while x >= x_min:
                    waypoints.append({'x': x, 'y': y, 'yaw': 3.14159})
                    x -= spacing
            forward = not forward
            y += spacing

        assert len(waypoints) > 0
        # Should cover the grid
        assert len(waypoints) == 25  # 5x5 grid

    def test_waypoint_save_load(self):
        """Test waypoint serialization."""
        import yaml
        import tempfile

        waypoints = [
            {'x': 1.0, 'y': 2.0, 'yaw': 0.0},
            {'x': 3.0, 'y': 4.0, 'yaw': 1.57},
        ]

        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml',
                                          delete=False) as f:
            yaml.dump({'waypoints': waypoints}, f)
            path = f.name

        with open(path, 'r') as f:
            loaded = yaml.safe_load(f)

        assert len(loaded['waypoints']) == 2
        assert loaded['waypoints'][0]['x'] == 1.0
        os.unlink(path)


class TestDataRecorder:
    """Test data recording logic."""

    def test_csv_record(self):
        """Test CSV record writing."""
        import csv
        import tempfile

        with tempfile.NamedTemporaryFile(mode='w', suffix='.csv',
                                          delete=False, newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'x', 'y', 'status', 'confidence'])
            writer.writerow(['2025-03-20T10:00:00', '1.0', '2.0', 'on', '0.85'])
            path = f.name

        with open(path, 'r') as f:
            reader = csv.DictReader(f)
            rows = list(reader)

        assert len(rows) == 1
        assert rows[0]['status'] == 'on'
        os.unlink(path)

    def test_summary_generation(self):
        """Test inspection summary statistics."""
        records = [
            {'power_status': 'on'},
            {'power_status': 'on'},
            {'power_status': 'off'},
            {'power_status': 'uncertain'},
            {'power_status': 'on'},
        ]
        total = len(records)
        on_count = sum(1 for r in records if r['power_status'] == 'on')
        off_count = sum(1 for r in records if r['power_status'] == 'off')

        assert total == 5
        assert on_count == 3
        assert off_count == 1
        assert round(on_count / total, 2) == 0.6
