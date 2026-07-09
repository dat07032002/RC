#!/usr/bin/env python3
"""
LiDAR Noise Characterization for Roboracer Phase 1
Measures range noise at different distances: 1m, 2m, 5m, 10m
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import sys

class LiDARNoiseTest(Node):
    def __init__(self):
        super().__init__('lidar_noise_test')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.ranges_history = []
        self.scan_count = 0
        self.current_distance = None

        self.get_logger().info("=" * 60)
        self.get_logger().info("LiDAR NOISE CHARACTERIZATION TEST")
        self.get_logger().info("=" * 60)
        self.get_logger().info("\nMeasure at these distances from a wall:")
        self.get_logger().info("  1. Position car at 1m from wall")
        self.get_logger().info("  2. Position car at 2m from wall")
        self.get_logger().info("  3. Position car at 5m from wall")
        self.get_logger().info("  4. Position car at 10m from wall")
        self.get_logger().info("\nEach position: collect 100 scans (~4 seconds)\n")

    def scan_callback(self, msg):
        """Collect range data from LiDAR scan."""
        # Get center rays (straight ahead, 10° cone)
        center_idx = len(msg.ranges) // 2
        center_range = 5  # ±5 rays from center

        center_ranges = []
        for i in range(center_idx - center_range, center_idx + center_range + 1):
            if 0 <= i < len(msg.ranges):
                r = msg.ranges[i]
                # Filter valid ranges
                if msg.range_min < r < msg.range_max and not np.isnan(r):
                    center_ranges.append(r)

        if center_ranges:
            self.ranges_history.extend(center_ranges)

        self.scan_count += 1

        # Print progress every 20 scans (~0.8 seconds)
        if self.scan_count % 20 == 0:
            if len(self.ranges_history) > 0:
                mean_range = np.mean(self.ranges_history)
                std_range = np.std(self.ranges_history)
                min_range = np.min(self.ranges_history)
                max_range = np.max(self.ranges_history)
                num_rays = len(self.ranges_history)

                self.get_logger().info(
                    f"Scans: {self.scan_count:3d} | "
                    f"Rays: {num_rays:3d} | "
                    f"Mean: {mean_range:.3f}m | "
                    f"Std: {std_range:.4f}m | "
                    f"Range: [{min_range:.3f}, {max_range:.3f}]"
                )

    def collect_at_distance(self, distance_m, num_scans=100):
        """Collect noise data at a specific distance."""
        self.ranges_history.clear()
        self.scan_count = 0
        self.current_distance = distance_m

        self.get_logger().info(f"\n{'=' * 60}")
        self.get_logger().info(f"Collecting at {distance_m}m from wall")
        self.get_logger().info(f"Target: {num_scans} scans (~4 seconds)")
        self.get_logger().info(f"{'=' * 60}")

        input("Position car, then press Enter to start...")

        # Collect for ~4 seconds (100 scans at 25Hz)
        start_scan = self.scan_count
        while self.scan_count - start_scan < num_scans:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Analyze
        if len(self.ranges_history) == 0:
            self.get_logger().error("No range data collected!")
            return

        ranges = np.array(self.ranges_history)
        mean_range = np.mean(ranges)
        std_range = np.std(ranges)
        min_range = np.min(ranges)
        max_range = np.max(ranges)
        outliers = np.sum((ranges < mean_range - 3*std_range) | (ranges > mean_range + 3*std_range))

        self.get_logger().info(f"\n{'RESULTS':^60}")
        self.get_logger().info(f"{'-' * 60}")
        self.get_logger().info(f"Distance: {distance_m}m")
        self.get_logger().info(f"Scans collected: {self.scan_count}")
        self.get_logger().info(f"Range rays: {len(ranges)}")
        self.get_logger().info(f"Mean distance: {mean_range:.4f}m")
        self.get_logger().info(f"Std deviation: {std_range:.4f}m (noise)")
        self.get_logger().info(f"Range: [{min_range:.4f}, {max_range:.4f}]")
        self.get_logger().info(f"Outliers (>3σ): {outliers} ({100*outliers/len(ranges):.1f}%)")

        print(f"\n→ Record in vehicle_params.yaml:")
        print(f"  distance_{distance_m}m:")
        print(f"    mean: {mean_range:.4f}")
        print(f"    std: {std_range:.4f}")

        return mean_range, std_range

    def run_test(self):
        """Run full noise characterization test."""
        results = {}

        for distance in [1, 2, 5, 10]:
            mean, std = self.collect_at_distance(distance, num_scans=100)
            results[distance] = (mean, std)
            time.sleep(1)  # Pause between distances

        # Analyze linear relationship
        self.get_logger().info(f"\n{'=' * 60}")
        self.get_logger().info("NOISE MODEL FITTING")
        self.get_logger().info(f"{'=' * 60}")

        distances = np.array(list(results.keys()))
        stds = np.array([std for _, std in results.values()])

        # Fit: noise_std = a + b * distance
        coeffs = np.polyfit(distances, stds, 1)
        b, a = coeffs  # slope, intercept

        self.get_logger().info(f"Linear model: noise_std = {a:.4f} + {b:.4f} * distance")
        self.get_logger().info(f"\nParameters for vehicle_params.yaml:")
        print(f"  noise_std_coefficient_a: {a:.4f}")
        print(f"  noise_std_coefficient_b: {b:.4f}")

        # Print all measurements
        print(f"\nAll measurements:")
        print(f"Distance (m) | Mean (m) | Std (m) | Error %")
        print(f"{'-' * 50}")
        for d in distances:
            mean, std = results[d]
            predicted = a + b * d
            error = abs(std - predicted) / std * 100 if std > 0 else 0
            print(f"    {d:6.1f}  |  {mean:.4f}  | {std:.4f} | {error:5.1f}%")

if __name__ == '__main__':
    import time
    rclpy.init()
    node = LiDARNoiseTest()

    try:
        node.run_test()
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user")
    finally:
        rclpy.shutdown()
