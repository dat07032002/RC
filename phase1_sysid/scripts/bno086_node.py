#!/usr/bin/env python3
"""Publish SparkFun BNO086 measurements as a ROS 2 sensor_msgs/Imu message."""

import math
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# JetPack 6.2 identifies the current carrier as an "Engineering Reference
# Developer Kit Super", which Blinka 9.1 does not auto-detect yet. The pin map is
# the standard Orin Nano/NX map. Preserve an explicit user override if one exists.
os.environ.setdefault('BLINKA_FORCEBOARD', 'JETSON_ORIN_NANO')
os.environ.setdefault('JETSON_MODEL_NAME', 'JETSON_ORIN_NANO')

try:
    from adafruit_bno08x import (
        BNO_REPORT_ACCELEROMETER,
        BNO_REPORT_GAME_ROTATION_VECTOR,
        BNO_REPORT_GYROSCOPE,
        BNO_REPORT_ROTATION_VECTOR,
    )
    from adafruit_bno08x.i2c import BNO08X_I2C
    from adafruit_extended_bus import ExtendedI2C
except ImportError as exc:
    raise SystemExit(
        "Missing BNO086 Python dependencies. Install with: "
        "python3 -m pip install --user adafruit-blinka adafruit-extended-bus "
        "adafruit-circuitpython-bno08x"
    ) from exc


class BNO086Node(Node):
    """Read the BNO086 over Jetson I2C and publish fused orientation + raw IMU."""

    def __init__(self):
        super().__init__('bno086_node')
        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('i2c_address', 0x4B)
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('topic', '/imu/data')
        self.declare_parameter('orientation_mode', 'game')

        self.bus_number = int(self.get_parameter('i2c_bus').value)
        self.address = int(self.get_parameter('i2c_address').value)
        self.publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.topic = str(self.get_parameter('topic').value)
        self.orientation_mode = str(
            self.get_parameter('orientation_mode').value).lower()

        if self.publish_rate <= 0.0 or self.publish_rate > 400.0:
            raise ValueError('publish_rate_hz must be in the range (0, 400]')
        if self.orientation_mode not in ('game', 'rotation'):
            raise ValueError("orientation_mode must be 'game' or 'rotation'")

        self.publisher = self.create_publisher(Imu, self.topic, 20)
        self.sensor = None
        self.last_reconnect_attempt = 0.0
        self.error_count = 0
        self._connect()
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish)

    def _connect(self):
        """Open the selected Linux I2C bus and configure BNO086 reports."""
        self.last_reconnect_attempt = time.monotonic()
        try:
            i2c = ExtendedI2C(self.bus_number)
            sensor = BNO08X_I2C(i2c, address=self.address)
            interval_us = max(2500, int(1_000_000 / self.publish_rate))
            sensor.enable_feature(BNO_REPORT_ACCELEROMETER, interval_us)
            sensor.enable_feature(BNO_REPORT_GYROSCOPE, interval_us)
            if self.orientation_mode == 'game':
                sensor.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, interval_us)
            else:
                sensor.enable_feature(BNO_REPORT_ROTATION_VECTOR, interval_us)
            self.sensor = sensor
            self.error_count = 0
            self.get_logger().info(
                f'BNO086 connected on /dev/i2c-{self.bus_number} at '
                f'0x{self.address:02x}; publishing {self.topic} at '
                f'{self.publish_rate:.1f} Hz ({self.orientation_mode} orientation)')
        except Exception as exc:
            self.sensor = None
            self.get_logger().error(
                f'BNO086 connection failed on I2C bus {self.bus_number}, '
                f'address 0x{self.address:02x}: {exc}')

    @staticmethod
    def _finite(values):
        return values is not None and all(math.isfinite(float(v)) for v in values)

    def _publish(self):
        if self.sensor is None:
            if time.monotonic() - self.last_reconnect_attempt >= 2.0:
                self._connect()
            return

        try:
            accel = self.sensor.acceleration
            gyro = self.sensor.gyro
            quaternion = (
                self.sensor.game_quaternion
                if self.orientation_mode == 'game'
                else self.sensor.quaternion
            )
            if not (self._finite(accel) and self._finite(gyro)
                    and self._finite(quaternion)):
                return

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            # Adafruit returns (i, j, k, real), equivalent to ROS (x, y, z, w).
            msg.orientation.x = float(quaternion[0])
            msg.orientation.y = float(quaternion[1])
            msg.orientation.z = float(quaternion[2])
            msg.orientation.w = float(quaternion[3])
            msg.angular_velocity.x = float(gyro[0])
            msg.angular_velocity.y = float(gyro[1])
            msg.angular_velocity.z = float(gyro[2])
            msg.linear_acceleration.x = float(accel[0])
            msg.linear_acceleration.y = float(accel[1])
            msg.linear_acceleration.z = float(accel[2])

            msg.orientation_covariance = [0.0] * 9
            msg.orientation_covariance[0] = 0.02 ** 2
            msg.orientation_covariance[4] = 0.02 ** 2
            msg.orientation_covariance[8] = (
                0.08 ** 2 if self.orientation_mode == 'game' else 0.04 ** 2)
            msg.angular_velocity_covariance = [0.0] * 9
            msg.angular_velocity_covariance[0] = 0.01 ** 2
            msg.angular_velocity_covariance[4] = 0.01 ** 2
            msg.angular_velocity_covariance[8] = 0.01 ** 2
            msg.linear_acceleration_covariance = [0.0] * 9
            msg.linear_acceleration_covariance[0] = 0.10 ** 2
            msg.linear_acceleration_covariance[4] = 0.10 ** 2
            msg.linear_acceleration_covariance[8] = 0.10 ** 2

            self.publisher.publish(msg)
            self.error_count = 0
        except Exception as exc:
            self.error_count += 1
            if self.error_count == 1:
                self.get_logger().warning(f'BNO086 read failed: {exc}')
            if self.error_count >= 10:
                self.get_logger().error('BNO086 repeatedly failed; reconnecting')
                self.sensor = None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BNO086Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
