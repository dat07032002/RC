#!/usr/bin/env python3
"""
IMU Calibration for Roboracer Phase 1
Measures gyro bias, accel bias, and noise
Car must be STATIONARY on flat ground
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import time

class IMUCalib(Node):
    def __init__(self):
        super().__init__('imu_calib')
        self.sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        self.gyro_readings = []
        self.accel_readings = []

        self.get_logger().info("=" * 60)
        self.get_logger().info("IMU CALIBRATION")
        self.get_logger().info("=" * 60)
        self.get_logger().info("\n⚠️  IMPORTANT: Car must be STATIONARY on FLAT ground")
        self.get_logger().info("   Do NOT move the car during calibration\n")

    def imu_callback(self, msg):
        """Record IMU readings."""
        gyro = [msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z]

        accel = [msg.linear_acceleration.x,
                 msg.linear_acceleration.y,
                 msg.linear_acceleration.z]

        self.gyro_readings.append(gyro)
        self.accel_readings.append(accel)

    def calibrate(self, duration=60):
        """Collect stationary IMU data for specified duration."""
        self.gyro_readings.clear()
        self.accel_readings.clear()

        self.get_logger().info(f"Collecting {duration} seconds of stationary IMU data...")
        input("Press Enter when car is positioned and stable...")

        start_time = time.time()
        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.1)

            if len(self.gyro_readings) % 20 == 0:
                elapsed = time.time() - start_time
                self.get_logger().info(f"  {elapsed:.1f}s / {duration}s")

        # Analyze
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("CALIBRATION RESULTS")
        self.get_logger().info("=" * 60)

        gyro_array = np.array(self.gyro_readings)
        accel_array = np.array(self.accel_readings)

        # Gyroscope
        gyro_mean = np.mean(gyro_array, axis=0)
        gyro_std = np.std(gyro_array, axis=0)

        self.get_logger().info("\nGYROSCOPE (angular velocity):")
        self.get_logger().info("  Bias (should be ~0 rad/s):")
        self.get_logger().info(f"    X: {gyro_mean[0]:+.6f} rad/s ({np.degrees(gyro_mean[0]):+.2f} deg/s)")
        self.get_logger().info(f"    Y: {gyro_mean[1]:+.6f} rad/s ({np.degrees(gyro_mean[1]):+.2f} deg/s)")
        self.get_logger().info(f"    Z: {gyro_mean[2]:+.6f} rad/s ({np.degrees(gyro_mean[2]):+.2f} deg/s)")

        self.get_logger().info("  Noise (std deviation):")
        self.get_logger().info(f"    X: {gyro_std[0]:.6f} rad/s")
        self.get_logger().info(f"    Y: {gyro_std[1]:.6f} rad/s")
        self.get_logger().info(f"    Z: {gyro_std[2]:.6f} rad/s")

        # Accelerometer
        accel_mean = np.mean(accel_array, axis=0)
        accel_std = np.std(accel_array, axis=0)
        accel_magnitude = np.linalg.norm(accel_mean)

        self.get_logger().info("\nACCELEROMETER (linear acceleration):")
        self.get_logger().info("  Bias (should be [0, 0, 9.81] m/s²):")
        self.get_logger().info(f"    X: {accel_mean[0]:+.4f} m/s²")
        self.get_logger().info(f"    Y: {accel_mean[1]:+.4f} m/s²")
        self.get_logger().info(f"    Z: {accel_mean[2]:+.4f} m/s² (gravity ~9.81)")

        self.get_logger().info("  Noise (std deviation):")
        self.get_logger().info(f"    X: {accel_std[0]:.4f} m/s²")
        self.get_logger().info(f"    Y: {accel_std[1]:.4f} m/s²")
        self.get_logger().info(f"    Z: {accel_std[2]:.4f} m/s²")

        # Sanity checks
        self.get_logger().info("\nSANITY CHECKS:")
        if abs(gyro_mean[0]) > 0.1 or abs(gyro_mean[1]) > 0.1 or abs(gyro_mean[2]) > 0.1:
            self.get_logger().warn("  ⚠️  Gyro bias is large! Check if car moved during test")
        else:
            self.get_logger().info("  ✓ Gyro bias looks good")

        if abs(accel_mean[2] - 9.81) > 0.5:
            self.get_logger().warn(f"  ⚠️  Z accel bias is {accel_mean[2]:.2f}, expected ~9.81")
        else:
            self.get_logger().info("  ✓ Accel bias looks good")

        # Print for vehicle_params.yaml
        print(f"\n→ Record in vehicle_params.yaml:")
        print(f"  gyro_bias: [{gyro_mean[0]:.6f}, {gyro_mean[1]:.6f}, {gyro_mean[2]:.6f}]")
        print(f"  gyro_noise_std: {np.mean(gyro_std):.6f}")
        print(f"  accel_bias: [{accel_mean[0]:.4f}, {accel_mean[1]:.4f}, {accel_mean[2]:.4f}]")
        print(f"  accel_noise_std: {np.mean(accel_std):.4f}")

if __name__ == '__main__':
    rclpy.init()
    node = IMUCalib()

    try:
        node.calibrate(duration=60)
    except KeyboardInterrupt:
        node.get_logger().info("Calibration interrupted")
    finally:
        rclpy.shutdown()
