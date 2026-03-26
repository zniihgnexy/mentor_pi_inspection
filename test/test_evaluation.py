#!/usr/bin/env python3
"""
System evaluation and test report generator.
Analyzes inspection data to produce test metrics required by the project:
- SLAM success rate and stability
- Patrol coverage rate
- Power LED detection accuracy
- System continuous run stability
"""
import csv
import json
import os
import sys
from datetime import datetime
from collections import Counter

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class InspectionEvaluator:
    """Evaluates inspection results and generates test report."""

    def __init__(self, data_dir=None):
        self.data_dir = data_dir or os.path.expanduser('~/inspection_data')
        self.records = []

    def load_csv(self, filepath: str):
        """Load inspection records from CSV."""
        with open(filepath, 'r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            self.records = list(reader)
        print(f'Loaded {len(self.records)} records from {filepath}')

    def load_latest(self):
        """Load the most recent CSV file from data directory."""
        csv_files = [f for f in os.listdir(self.data_dir) if f.endswith('.csv')]
        if not csv_files:
            print('No CSV files found')
            return
        latest = sorted(csv_files)[-1]
        self.load_csv(os.path.join(self.data_dir, latest))


    def evaluate_detection_accuracy(self, ground_truth: dict = None) -> dict:
        """
        Evaluate power LED detection accuracy.
        ground_truth: {waypoint_index: 'on'/'off'} mapping of actual states.
        If no ground truth, just report distribution.
        """
        if not self.records:
            return {'error': 'no records'}

        statuses = [r.get('power_status', 'unknown') for r in self.records]
        counter = Counter(statuses)
        total = len(statuses)

        result = {
            'total_detections': total,
            'on_count': counter.get('on', 0),
            'off_count': counter.get('off', 0),
            'uncertain_count': counter.get('uncertain', 0),
            'on_ratio': round(counter.get('on', 0) / total, 3) if total else 0,
            'off_ratio': round(counter.get('off', 0) / total, 3) if total else 0,
            'uncertain_ratio': round(counter.get('uncertain', 0) / total, 3) if total else 0,
        }

        if ground_truth:
            correct = 0
            evaluated = 0
            for r in self.records:
                wp_idx = r.get('waypoint_index', '-1')
                if str(wp_idx) in ground_truth:
                    evaluated += 1
                    if r.get('power_status') == ground_truth[str(wp_idx)]:
                        correct += 1
            result['ground_truth_evaluated'] = evaluated
            result['correct'] = correct
            result['accuracy'] = round(correct / evaluated, 3) if evaluated else 0

        return result

    def evaluate_patrol_coverage(self) -> dict:
        """Evaluate how many waypoints were actually visited."""
        if not self.records:
            return {'error': 'no records'}

        wp_indices = set()
        for r in self.records:
            idx = r.get('waypoint_index', '-1')
            if idx != '-1' and idx != '':
                wp_indices.add(int(idx))

        return {
            'waypoints_visited': len(wp_indices),
            'waypoint_indices': sorted(wp_indices),
        }

    def evaluate_stability(self) -> dict:
        """Evaluate system run stability from timestamps."""
        if not self.records:
            return {'error': 'no records'}

        timestamps = []
        for r in self.records:
            try:
                ts = datetime.fromisoformat(r['timestamp'])
                timestamps.append(ts)
            except (ValueError, KeyError):
                continue

        if len(timestamps) < 2:
            return {'total_records': len(timestamps), 'duration_seconds': 0}

        timestamps.sort()
        duration = (timestamps[-1] - timestamps[0]).total_seconds()

        # Check for gaps (potential crashes)
        gaps = []
        for i in range(1, len(timestamps)):
            gap = (timestamps[i] - timestamps[i-1]).total_seconds()
            if gap > 30:  # gap > 30s might indicate issue
                gaps.append({
                    'between_records': [i-1, i],
                    'gap_seconds': round(gap, 1)
                })

        return {
            'total_records': len(timestamps),
            'duration_seconds': round(duration, 1),
            'duration_minutes': round(duration / 60, 1),
            'avg_interval_seconds': round(duration / (len(timestamps) - 1), 1),
            'suspicious_gaps': len(gaps),
            'gap_details': gaps,
        }


    def generate_report(self, output_path: str = None,
                        ground_truth: dict = None,
                        total_waypoints: int = 0) -> str:
        """Generate a complete evaluation report."""
        detection = self.evaluate_detection_accuracy(ground_truth)
        coverage = self.evaluate_patrol_coverage()
        stability = self.evaluate_stability()

        if total_waypoints > 0 and 'waypoints_visited' in coverage:
            coverage['total_planned'] = total_waypoints
            coverage['coverage_rate'] = round(
                coverage['waypoints_visited'] / total_waypoints, 3)

        report_lines = [
            '=' * 60,
            '  计算机教室巡检机器人 - 系统测试评估报告',
            '=' * 60,
            f'  生成时间: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}',
            '',
            '-' * 60,
            '  1. 电源状态识别准确率',
            '-' * 60,
            f'  总检测次数: {detection.get("total_detections", 0)}',
            f'  开机检测: {detection.get("on_count", 0)} '
            f'({detection.get("on_ratio", 0):.1%})',
            f'  关机检测: {detection.get("off_count", 0)} '
            f'({detection.get("off_ratio", 0):.1%})',
            f'  不确定: {detection.get("uncertain_count", 0)} '
            f'({detection.get("uncertain_ratio", 0):.1%})',
        ]

        if 'accuracy' in detection:
            report_lines.extend([
                f'  与真实值对比: {detection["correct"]}/{detection["ground_truth_evaluated"]}',
                f'  识别准确率: {detection["accuracy"]:.1%}',
            ])

        report_lines.extend([
            '',
            '-' * 60,
            '  2. 巡航覆盖率',
            '-' * 60,
            f'  已访问路径点: {coverage.get("waypoints_visited", 0)}',
        ])

        if 'coverage_rate' in coverage:
            report_lines.append(
                f'  计划路径点: {coverage.get("total_planned", 0)}')
            report_lines.append(
                f'  覆盖率: {coverage["coverage_rate"]:.1%}')

        report_lines.extend([
            '',
            '-' * 60,
            '  3. 系统连续运行稳定性',
            '-' * 60,
            f'  总记录数: {stability.get("total_records", 0)}',
            f'  运行时长: {stability.get("duration_minutes", 0)} 分钟',
            f'  平均检测间隔: {stability.get("avg_interval_seconds", 0)} 秒',
            f'  异常中断次数: {stability.get("suspicious_gaps", 0)}',
        ])

        if stability.get('gap_details'):
            for gap in stability['gap_details']:
                report_lines.append(
                    f'    - 记录 {gap["between_records"]} 间隔 '
                    f'{gap["gap_seconds"]}s')

        report_lines.extend([
            '',
            '=' * 60,
            '  评估结论',
            '=' * 60,
        ])

        # Auto conclusion
        issues = []
        if detection.get('uncertain_ratio', 0) > 0.3:
            issues.append('不确定检测比例偏高，建议调整LED检测参数')
        if coverage.get('coverage_rate', 1) < 0.8:
            issues.append('巡航覆盖率不足80%，建议检查路径规划')
        if stability.get('suspicious_gaps', 0) > 2:
            issues.append('存在多次异常中断，建议检查系统稳定性')

        if not issues:
            report_lines.append('  系统各项指标正常，测试通过。')
        else:
            for issue in issues:
                report_lines.append(f'  ⚠ {issue}')

        report_lines.append('=' * 60)
        report = '\n'.join(report_lines)

        if output_path:
            os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
            with open(output_path, 'w', encoding='utf-8') as f:
                f.write(report)
            print(f'Report saved to {output_path}')

        return report


def main():
    """CLI entry point for evaluation."""
    import argparse
    parser = argparse.ArgumentParser(description='Inspection evaluation tool')
    parser.add_argument('--data', '-d', help='CSV data file path')
    parser.add_argument('--output', '-o', help='Report output path')
    parser.add_argument('--waypoints', '-w', type=int, default=0,
                        help='Total planned waypoints')
    parser.add_argument('--ground-truth', '-g', help='Ground truth JSON file')
    args = parser.parse_args()

    evaluator = InspectionEvaluator()

    if args.data:
        evaluator.load_csv(args.data)
    else:
        evaluator.load_latest()

    gt = None
    if args.ground_truth and os.path.exists(args.ground_truth):
        with open(args.ground_truth, 'r') as f:
            gt = json.load(f)

    report = evaluator.generate_report(
        output_path=args.output,
        ground_truth=gt,
        total_waypoints=args.waypoints
    )
    print(report)


if __name__ == '__main__':
    main()
