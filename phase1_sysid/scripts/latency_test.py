#!/usr/bin/env python3
"""
End-to-End Latency Profiling for Roboracer Phase 1
Measures LiDAR → SLAM → Policy → Motor delay
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import numpy as np
import time

class LatencyTest(Node):
    def __init__(self):
        super().__init__('latency_test')
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        self.scan_timestamps = []
        self.odom_timestamps = []
        self.scan_count = 0
        self.odom_count = 0

        self.get_logger().info("=" * 60)
        self.get_logger().info("END-TO-END LATENCY TEST")
        self.get_logger().info("=" * 60)
        self.get_logger().info("\nMeasures time from:")
        self.get_logger().info("  /scan timestamp → /odom timestamp")
        self.get_logger().info("\nDrive car slowly around maze for ~5 minutes\n")

    def scan_callback(self, msg):
        """Record LiDAR scan timestamp."""
        now = self.get_clock().now()
        self.scan_timestamps.append({
            'timestamp': now,
            'msg_time': msg.header.stamp,
        })
        self.scan_count += 1

    def odom_callback(self, msg):
        """Record odometry timestamp."""
        now = self.get_clock().now()
        self.odom_timestamps.append({
            'timestamp': now,
            'msg_time': msg.header.stamp,
        })
        self.odom_count += 1

        if self.odom_count % 50 == 0:
            self.get_logger().info(f"Scans: {self.scan_count}, Odometry: {self.odom_count}")

    def analyze_latency(self):
        """Analyze collected timestamps."""
        if len(self.scan_timestamps) < 10 or len(self.odom_timestamps) < 10:
            self.get_logger().error("Not enough data collected!")
            return

        self.get_logger().info(f"\n{'=' * 60}")
        self.get_logger().info("LATENCY ANALYSIS")
        self.get_logger().info(f"{'=' * 60}")

        # Simple latency: odom_time - scan_time for nearest pairs
        latencies = []

        for odom_msg in self.odom_timestamps:
            # Find nearest scan before this odom
            odom_ros_time = rclpy.time.Time.from_msg(odom_msg['msg_time']).nanoseconds

            for scan_msg in self.scan_timestamps:
                scan_ros_time = rclpy.time.Time.from_msg(scan_msg['msg_time']).nanoseconds

                if scan_ros_time <= odom_ros_time:
                    latency_ns = odom_ros_time - scan_ros_time
                    latency_ms = latency_ns / 1e6
                    latencies.append(latency_ms)

        if len(latencies) > 0:
            latencies = np.array(latencies)
            # Remove outliers (>500ms, probably lost scans)
            latencies = latencies[latencies < 500]

            self.get_logger().info(f"\nLiDAR → SLAM latency:")
            self.get_logger().info(f"  Mean: {np.mean(latencies):.2f} ms")
            self.get_logger().info(f"  Median: {np.median(latencies):.2f} ms")
            self.get_logger().info(f"  Std: {np.std(latencies):.2f} ms")
            self.get_logger().info(f"  Min: {np.min(latencies):.2f} ms")
            self.get_logger().info(f"  Max: {np.max(latencies):.2f} ms (might be outlier)")

            print(f"\n→ Record in vehicle_params.yaml:")
            print(f"  lidar_to_slam_ms: {np.mean(latencies):.0f}")

            # LiDAR frequency check
            if len(self.scan_timestamps) > 1:
                scan_intervals = []
                for i in range(1, min(100, len(self.scan_timestamps))):
                    t1 = rclpy.time.Time.from_msg(self.scan_timestamps[i-1]['msg_time']).nanoseconds
                    t2 = rclpy.time.Time.from_msg(self.scan_timestamps[i]['msg_time']).nanoseconds
                    interval_ms = (t2 - t1) / 1e6
                    scan_intervals.append(interval_ms)

                if scan_intervals:
                    mean_interval = np.mean(scan_intervals)
                    freq = 1000 / mean_interval if mean_interval > 0 else 0
                    self.get_logger().info(f"\nLiDAR frequency:")
                    self.get_logger().info(f"  Measured: {freq:.1f} Hz (expected 25 Hz)")

if __name__ == '__main__':
    rclpy.init()
    node = LatencyTest()

    try:
        print("Collecting latency data... (press Ctrl+C to stop)\n")
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\nAnalyzing collected data...")
        node.analyze_latency()
    finally:
        rclpy.shutdown()
