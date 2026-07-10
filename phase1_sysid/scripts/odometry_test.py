#!/usr/bin/env python3
"""
Odometry distance calibration for Phase 1.

Drives straight for a fixed duration at a low capped speed, then reports the
distance measured by odometry. Measure the physical distance on the floor and
compute correction = physical_distance / odom_distance.
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt16


class OdometryTest(Node):
    def __init__(self, odom_topic):
        super().__init__('odometry_test')
        self.pub_motor = self.create_publisher(UInt16, '/motor/command', 10)
        self.pub_servo = self.create_publisher(UInt16, '/servo/command', 10)
        self.sub_odom = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        self.start_pose = None
        self.last_pose = None
        self.start_time = None
        self.last_time = None
        self.last_velocity = 0.0
        self.odom_topic = odom_topic

        self.get_logger().info("=" * 60)
        self.get_logger().info("ODOMETRY DISTANCE CALIBRATION")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Odometry topic: {odom_topic}")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        now = time.time()
        self.last_pose = (x, y)
        self.last_time = now
        self.last_velocity = msg.twist.twist.linear.x

        if self.start_pose is None:
            self.start_pose = (x, y)
            self.start_time = now

    def reset_start(self):
        self.start_pose = self.last_pose
        self.start_time = time.time()

    def distance(self):
        if self.start_pose is None or self.last_pose is None:
            return 0.0
        dx = self.last_pose[0] - self.start_pose[0]
        dy = self.last_pose[1] - self.start_pose[1]
        return math.hypot(dx, dy)

    def stop(self):
        for _ in range(10):
            self.pub_motor.publish(UInt16(data=1500))
            self.pub_servo.publish(UInt16(data=1500))
            rclpy.spin_once(self, timeout_sec=0.02)

    def run_drive(self, duration, motor_pwm, servo_pwm, real_distance):
        self.get_logger().info("Centering steering and waiting for odom...")
        wait_start = time.time()
        while self.last_pose is None and time.time() - wait_start < 5.0:
            self.pub_servo.publish(UInt16(data=servo_pwm))
            self.pub_motor.publish(UInt16(data=1500))
            rclpy.spin_once(self, timeout_sec=0.05)

        if self.last_pose is None:
            self.get_logger().error(f"No odometry received on {self.odom_topic}")
            return 1

        self.stop()
        time.sleep(0.3)
        self.reset_start()

        self.get_logger().info(
            f"Driving straight for {duration:.1f}s: servo PWM {servo_pwm}, motor PWM {motor_pwm}")

        drive_start = time.time()
        while time.time() - drive_start < duration:
            self.pub_servo.publish(UInt16(data=servo_pwm))
            self.pub_motor.publish(UInt16(data=motor_pwm))
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().info("Stopping...")
        self.stop()
        settle_start = time.time()
        while time.time() - settle_start < 1.0:
            self.pub_motor.publish(UInt16(data=1500))
            self.pub_servo.publish(UInt16(data=servo_pwm))
            rclpy.spin_once(self, timeout_sec=0.05)

        odom_distance = self.distance()
        elapsed = max((self.last_time or time.time()) - (self.start_time or time.time()), 1e-6)
        avg_odom_velocity = odom_distance / elapsed

        print("\nResults:")
        print(f"  odom_distance_m: {odom_distance:.3f}")
        print(f"  elapsed_s: {elapsed:.2f}")
        print(f"  avg_odom_velocity_mps: {avg_odom_velocity:.3f}")
        print(f"  final_odom_velocity_mps: {self.last_velocity:.3f}")

        if real_distance is not None:
            correction = real_distance / odom_distance if odom_distance > 1e-6 else float('nan')
            print("\nRecord in vehicle_params.yaml:")
            print(f"  distance_correction_factor: {correction:.3f}")
            print(f"  velocity_correction_factor: {correction:.3f}")
        else:
            print("\nMeasure the physical floor distance from start to stop, then compute:")
            print("  distance_correction_factor = physical_distance_m / odom_distance_m")
            print("  velocity_correction_factor = same value for now")

        return 0


def main():
    parser = argparse.ArgumentParser(description='Straight-line odometry calibration')
    parser.add_argument('--duration', type=float, default=8.0,
                        help='Seconds to drive straight')
    parser.add_argument('--motor-pwm', type=int, default=2000,
                        help='Motor PWM command; speed is still capped by bringup max_speed_mps')
    parser.add_argument('--servo-pwm', type=int, default=1500,
                        help='Servo PWM command; 1500 maps to measured straight center')
    parser.add_argument('--odom-topic', default='/odometry/filtered',
                        help='Odometry topic to read, e.g. --odom-topic /odom')
    parser.add_argument('--real-distance', type=float, default=None,
                        help='Measured physical distance in meters, if known after a run')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = OdometryTest(args.odom_topic)
    try:
        return node.run_drive(args.duration, args.motor_pwm, args.servo_pwm, args.real_distance)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted; stopping motor")
        node.stop()
        return 130
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
