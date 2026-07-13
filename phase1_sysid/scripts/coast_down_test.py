#!/usr/bin/env python3
"""
Coast-down friction test (Phase 1) — the sim's #1 sim-to-real lever.

WHY THIS EXISTS (not throttle_test.py's coast-down):
  The VESC runs in SPEED (eRPM) control. Commanding 0 eRPM makes it ACTIVELY
  BRAKE to a stop — that measures braking, not friction. A real coast-down needs
  the motor to FREEWHEEL, i.e. zero torque. This script spins up in speed mode,
  then switches to CURRENT control at 0.0 A (no torque) so the wheels roll free,
  and logs /odom velocity decay. The decay slope = tire + rolling + aero drag,
  which maps to TIRE_FRICTION / the ground friction in the sim.

RUN THE STACK WITHOUT THE BRIDGE (its watchdog would re-brake the motor):
  ros2 launch ~/RC/phase1_sysid/config/sysid_bringup.launch.py \
      use_bridge:=false
  # then, in another terminal:
  python3 coast_down_test.py --target-mps 2.5 --odom-topic /odom

SAFETY:
  * Talks to the VESC directly (/commands/motor/{speed,current}); it has its own
    speed cap via --target-mps. Keep it LOW (this car does 60 mph at full power).
  * Ctrl-C brakes the motor to a stop (speed 0) immediately.
  * NEEDS A LONG CLEAR RUNOUT — freewheeling rolls much farther than braking.
"""

import argparse
import csv
import statistics
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class CoastDownTest(Node):
    def __init__(self, target_mps, gain, motor_sign, spinup_s, coast_timeout_s,
                 odom_topic, imu_topic, output_prefix):
        super().__init__('coast_down_test')
        self.pub_speed = self.create_publisher(Float64, '/commands/motor/speed', 10)
        self.pub_current = self.create_publisher(Float64, '/commands/motor/current', 10)
        # vesc_to_odom has use_servo_cmd_to_calc_angular_velocity: true, so /odom
        # ONLY publishes while a servo command is present. Hold center throughout.
        self.pub_servo = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.create_subscription(Imu, imu_topic, self.on_imu, 50)

        self.target_erpm = target_mps * gain * motor_sign
        self.spinup_s = spinup_s
        self.coast_timeout_s = coast_timeout_s
        self.target_mps = target_mps
        self.output_prefix = output_prefix

        self.vel = []
        self.t = []
        self.t0 = None
        self.imu_samples = []
        self.imu_baseline = None
        self.imu_forward_axis = None
        self.imu_forward_sign = 1.0
        self.spinup_imu_idx = 0
        self.coast_imu_idx = 0
        self.coast_wall_t0 = None

        self.get_logger().info("=" * 60)
        self.get_logger().info("COAST-DOWN (FREEWHEEL) FRICTION TEST")
        self.get_logger().info("=" * 60)
        self.get_logger().info(
            f"target={target_mps} m/s (={self.target_erpm:.0f} eRPM), "
            f"spinup={spinup_s}s, coast_timeout={coast_timeout_s}s")
        self.get_logger().info("Ensure a LONG clear runout. Ctrl-C brakes to stop.\n")

    def on_odom(self, msg):
        v = abs(msg.twist.twist.linear.x)
        now = time.time()
        if self.t0 is None:
            self.t0 = now
        self.vel.append(v)
        self.t.append(now - self.t0)

    def on_imu(self, msg):
        self.imu_samples.append((
            time.time(),
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ))

    def _speed(self, erpm):
        self.pub_speed.publish(Float64(data=float(erpm)))

    def _current(self, amps):
        self.pub_current.publish(Float64(data=float(amps)))

    def _servo_center(self):
        # 0.55 = calibrated straight-ahead (from bridge mapping); keeps /odom alive
        self.pub_servo.publish(Float64(data=0.55))

    def _measure_imu_baseline(self, duration=1.0):
        """Measure gravity/mount offsets while holding the stopped car still."""
        start_idx = len(self.imu_samples)
        self.get_logger().info(
            f"Measuring stationary BNO086 baseline for {duration:.1f}s...")
        start = time.time()
        while time.time() - start < duration:
            self._servo_center()
            self._speed(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
        samples = self.imu_samples[start_idx:]
        if len(samples) < 10:
            self.get_logger().warn(
                "Not enough BNO086 samples; continuing with odometry only.")
            return
        self.imu_baseline = tuple(
            statistics.mean(sample[axis] for sample in samples)
            for axis in (1, 2, 3)
        )
        self.get_logger().info(
            "BNO086 baseline: "
            f"[{self.imu_baseline[0]:+.4f}, {self.imu_baseline[1]:+.4f}, "
            f"{self.imu_baseline[2]:+.4f}] m/s^2 ({len(samples)} samples)")

    def _detect_forward_imu_axis(self):
        """Infer mounted forward axis/sign from the acceleration phase."""
        if self.imu_baseline is None:
            return
        samples = self.imu_samples[self.spinup_imu_idx:self.coast_imu_idx]
        if len(samples) < 10:
            self.get_logger().warn("Not enough spin-up IMU samples to detect forward axis.")
            return
        means = [
            statistics.mean(sample[axis + 1] - self.imu_baseline[axis]
                            for sample in samples)
            for axis in (0, 1)
        ]
        axis = max(range(2), key=lambda idx: abs(means[idx]))
        self.imu_forward_axis = axis
        self.imu_forward_sign = 1.0 if means[axis] >= 0.0 else -1.0
        axis_name = ('x', 'y')[axis]
        self.get_logger().info(
            f"Detected vehicle-forward IMU axis: {self.imu_forward_sign:+.0f}{axis_name} "
            f"(spin-up mean {abs(means[axis]):.3f} m/s^2)")

    def run(self):
        self._measure_imu_baseline()

        # --- spin up to target in speed mode ---
        self.get_logger().info(f"Accelerating to {self.target_mps} m/s...")
        self.spinup_imu_idx = len(self.imu_samples)
        start = time.time()
        while time.time() - start < self.spinup_s:
            self._servo_center()
            self._speed(self.target_erpm)
            rclpy.spin_once(self, timeout_sec=0.02)

        # --- mark the coast start, then FREEWHEEL (0 torque) ---
        coast_start_idx = len(self.vel)
        coast_t0 = time.time() - self.t0 if self.t0 else 0.0
        self.coast_imu_idx = len(self.imu_samples)
        self.coast_wall_t0 = time.time()
        self._detect_forward_imu_axis()
        self.get_logger().info("FREEWHEEL (current=0) — coasting...")
        start = time.time()
        # publish current=0 continuously so the VESC stays in current mode at 0 torque
        while time.time() - start < self.coast_timeout_s:
            self._servo_center()
            self._current(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
            if len(self.vel) > coast_start_idx + 5 and self.vel[-1] < 0.05:
                self.get_logger().info("Car stopped.")
                break

        self.stop()
        self.report(coast_start_idx, coast_t0)

    def stop(self):
        # brake to a clean stop for safety
        for _ in range(10):
            self._speed(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def report(self, coast_start_idx, coast_t0):
        if len(self.vel) - coast_start_idx < 5:
            self.get_logger().error("Not enough coast data — check /odom is publishing.")
            return
        ct = [t - coast_t0 for t in self.t[coast_start_idx:]]
        cv = self.vel[coast_start_idx:]
        v_peak = max(cv[:5])

        # crude overall slope (peak -> near-zero), for a quick number only
        # (the real fit uses the full curve below — decel is speed-dependent)
        i_lo = next((i for i, v in enumerate(cv) if v < 0.10), len(cv) - 1)
        elapsed_from_first_sample = ct[i_lo] - ct[0]
        if elapsed_from_first_sample > 0:
            decel_magnitude = (
                (cv[0] - cv[i_lo]) / elapsed_from_first_sample)
        else:
            decel_magnitude = 0.0

        self.get_logger().info("\nCoast-down results:")
        self.get_logger().info(f"  Peak (coast start): {v_peak:.2f} m/s")
        self.get_logger().info(
            f"  Avg decel magnitude: {decel_magnitude:.2f} m/s^2")
        print("\n-> Send me this curve; I'll fit rolling vs aero drag "
              "(decel vs speed) for TIRE_FRICTION.")
        print(f"\nCoast-down curve (peak {v_peak:.2f} m/s):")
        print("t_coast(s)\tv(m/s)")
        for t, v in zip(ct, cv):
            print(f"{t:.3f}\t{v:.3f}")

        imu_curve = []
        if self.imu_baseline is not None and self.imu_forward_axis is not None:
            sample_axis = self.imu_forward_axis + 1
            baseline = self.imu_baseline[self.imu_forward_axis]
            for sample in self.imu_samples[self.coast_imu_idx:]:
                t_coast = sample[0] - self.coast_wall_t0
                accel = self.imu_forward_sign * (sample[sample_axis] - baseline)
                imu_curve.append((t_coast, accel))

            active_decel = [
                accel for t, accel in imu_curve
                if t >= 0.10 and accel < -0.10
            ]
            if active_decel:
                median_decel = -statistics.median(active_decel)
                mean_decel = -statistics.mean(active_decel)
                self.get_logger().info(
                    f"  BNO active-decel median: {median_decel:.3f} m/s^2")
                self.get_logger().info(
                    f"  BNO active-decel mean:   {mean_decel:.3f} m/s^2 "
                    f"({len(active_decel)} samples)")
            else:
                self.get_logger().warn(
                    "No BNO086 samples crossed the -0.10 m/s^2 decel threshold.")

        if self.output_prefix:
            odom_path = self.output_prefix + '_odom.csv'
            with open(odom_path, 'w', newline='', encoding='utf-8') as stream:
                writer = csv.writer(stream)
                writer.writerow(('t_coast_s', 'velocity_mps'))
                writer.writerows((f'{t:.6f}', f'{v:.6f}') for t, v in zip(ct, cv))
            self.get_logger().info(f"Saved odometry curve: {odom_path}")

            if imu_curve:
                imu_path = self.output_prefix + '_imu.csv'
                with open(imu_path, 'w', newline='', encoding='utf-8') as stream:
                    writer = csv.writer(stream)
                    writer.writerow(('t_coast_s', 'forward_accel_mps2'))
                    writer.writerows(
                        (f'{t:.6f}', f'{a:.6f}') for t, a in imu_curve)
                self.get_logger().info(f"Saved BNO086 curve: {imu_path}")


def main():
    p = argparse.ArgumentParser(description="Freewheel coast-down friction test")
    p.add_argument('--target-mps', type=float, default=2.5,
                   help='coast-start speed (KEEP LOW — 60 mph car)')
    p.add_argument('--gain', type=float, default=4614.0,
                   help='speed_to_erpm_gain (from vesc.yaml / bridge)')
    p.add_argument('--motor-sign', type=float, default=-1.0,
                   help='+PWM was physically reverse -> forward is negative eRPM')
    p.add_argument('--spinup', type=float, default=3.0, help='seconds to reach speed')
    p.add_argument('--coast-timeout', type=float, default=8.0,
                   help='max seconds to log the coast')
    p.add_argument('--odom-topic', default='/odom')
    p.add_argument('--imu-topic', default='/imu/data')
    p.add_argument('--output-prefix', default='',
                   help='write <prefix>_odom.csv and <prefix>_imu.csv')
    args, ros_args = p.parse_known_args()

    rclpy.init(args=ros_args)
    node = CoastDownTest(args.target_mps, args.gain, args.motor_sign,
                         args.spinup, args.coast_timeout, args.odom_topic,
                         args.imu_topic, args.output_prefix)

    print("\nWaiting for first odometry message...")
    t_wait = time.time()
    while not node.vel and time.time() - t_wait < 3.0:
        # vesc_to_odom only publishes while a servo command is present. Send the
        # calibrated center command during startup to avoid waiting forever for
        # an odometry message that cannot otherwise be produced.
        node._servo_center()
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.vel:
        node.get_logger().warn(
            "No odometry! Is the stack up? (launch with use_bridge:=false)")
        rclpy.shutdown()
        return

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted — braking to stop.")
        node.stop()
    finally:
        node.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
