#!/usr/bin/env python3
"""
Throttle Response Testing for Roboracer Phase 1
Measures max velocity, acceleration, and response delay
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16
from nav_msgs.msg import Odometry
import numpy as np
import time
import sys

class ThrottleTest(Node):
    def __init__(self):
        super().__init__('throttle_test')
        self.pub_throttle = self.create_publisher(UInt16, '/motor/command', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        self.velocities = []
        self.times = []
        self.start_time = None

        self.get_logger().info("=" * 60)
        self.get_logger().info("THROTTLE RESPONSE TEST")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Will test throttle levels: 25%, 50%, 75%, 100%")
        self.get_logger().info("Duration per test: 5 seconds")
        self.get_logger().info("\nEnsure clear track! Car will accelerate.\n")

    def odom_callback(self, msg):
        """Record velocity and timestamp."""
        vel = msg.twist.twist.linear.x
        if self.start_time is None:
            self.start_time = time.time()

        elapsed = time.time() - self.start_time
        self.velocities.append(vel)
        self.times.append(elapsed)

    def test_throttle_level(self, throttle_pct, duration=5):
        """Test at constant throttle level."""
        self.velocities.clear()
        self.times.clear()
        self.start_time = None

        pwm = int(1500 + (throttle_pct / 100) * 500)
        self.get_logger().info(f"\n{'=' * 60}")
        self.get_logger().info(f"Testing Throttle {throttle_pct}% (PWM {pwm} µs)")
        self.get_logger().info(f"{'=' * 60}")

        # Give time for subscription to work
        time.sleep(0.5)

        start = time.time()
        self.get_logger().info("Accelerating...")

        # spin_once processes odom_callback AND provides the ~50ms pacing
        while time.time() - start < duration:
            self.pub_throttle.publish(UInt16(data=pwm))
            rclpy.spin_once(self, timeout_sec=0.05)

        # Stop
        self.get_logger().info("Stopping...")
        self.pub_throttle.publish(UInt16(data=1500))
        time.sleep(0.5)

        # Analyze
        if len(self.velocities) == 0:
            self.get_logger().error("No velocity data received! Check /odometry/filtered topic")
            return

        max_vel = max(self.velocities)
        min_vel = min(self.velocities)

        # Calculate acceleration
        if len(self.velocities) > 1:
            vel_change = self.velocities[-1] - self.velocities[0]
            time_change = self.times[-1] - self.times[0]
            avg_accel = vel_change / time_change if time_change > 0 else 0
        else:
            avg_accel = 0

        self.get_logger().info(f"\nResults for {throttle_pct}%:")
        self.get_logger().info(f"  Min velocity: {min_vel:.2f} m/s")
        self.get_logger().info(f"  Max velocity: {max_vel:.2f} m/s")
        self.get_logger().info(f"  Avg acceleration: {avg_accel:.2f} m/s²")

        # Print raw data for spreadsheet
        print(f"\n→ Record in vehicle_params.yaml:")
        print(f"  max_velocity: {max_vel:.2f}  # from 100% throttle test")
        print(f"  max_acceleration: {avg_accel:.2f}  # from acceleration phase")

        # Print velocity curve
        print(f"\nVelocity curve (for spreadsheet):")
        print("Time (s)\tVelocity (m/s)")
        for t, v in zip(self.times[::2], self.velocities[::2]):  # Print every other point
            print(f"{t:.2f}\t{v:.2f}")

    def test_coast_down(self):
        """Measure deceleration when throttle returns to neutral."""
        self.velocities.clear()
        self.times.clear()
        self.start_time = None

        self.get_logger().info(f"\n{'=' * 60}")
        self.get_logger().info("COAST-DOWN TEST (measure friction/drag)")
        self.get_logger().info(f"{'=' * 60}")

        # Accelerate to max
        self.get_logger().info("Accelerating to max speed...")
        for _ in range(100):  # 5 seconds at 20Hz
            self.pub_throttle.publish(UInt16(data=2000))
            rclpy.spin_once(self, timeout_sec=0.05)

        # Coast down
        self.get_logger().info("Coasting down (0% throttle)...")
        start = time.time()

        while time.time() - start < 10 and len(self.velocities) < 200:
            self.pub_throttle.publish(UInt16(data=1500))  # Neutral throttle
            rclpy.spin_once(self, timeout_sec=0.05)

        if len(self.velocities) > 10:
            max_vel = max(self.velocities[:50])  # Peak from acceleration
            final_vel = self.velocities[-1]

            # Fit deceleration
            vel_drop = max_vel - final_vel
            time_elapsed = self.times[-1]
            friction_accel = -vel_drop / time_elapsed if time_elapsed > 0 else 0

            self.get_logger().info(f"\nCoast-down results:")
            self.get_logger().info(f"  Peak velocity: {max_vel:.2f} m/s")
            self.get_logger().info(f"  Final velocity: {final_vel:.2f} m/s")
            self.get_logger().info(f"  Deceleration: {friction_accel:.2f} m/s²")

            print(f"\n→ Record in vehicle_params.yaml:")
            print(f"  friction_coefficient: {friction_accel:.2f}  # negative = deceleration")

if __name__ == '__main__':
    rclpy.init()
    node = ThrottleTest()

    print("\nWaiting for first odometry message...")
    _wait_start = time.time()
    while not node.velocities and time.time() - _wait_start < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    if not node.velocities:
        node.get_logger().warn("No odometry received! Ensure /odometry/filtered is publishing")
        print("Commands:")
        print("  1. Check: ros2 topic list | grep odom")
        print("  2. Monitor: ros2 topic echo /odometry/filtered")
        rclpy.shutdown()
        sys.exit(1)

    try:
        # Test at different throttle levels
        for pct in [25, 50, 75, 100]:
            node.test_throttle_level(pct, duration=5)
            time.sleep(2)  # Rest between tests

        # Coast-down test
        node.test_coast_down()

    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user")
    finally:
        # Ensure motor stops
        node.pub_throttle.publish(UInt16(data=1500))
        rclpy.shutdown()
