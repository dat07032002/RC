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
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


class CoastDownTest(Node):
    def __init__(self, target_mps, gain, motor_sign, spinup_s, coast_timeout_s,
                 odom_topic):
        super().__init__('coast_down_test')
        self.pub_speed = self.create_publisher(Float64, '/commands/motor/speed', 10)
        self.pub_current = self.create_publisher(Float64, '/commands/motor/current', 10)
        # vesc_to_odom has use_servo_cmd_to_calc_angular_velocity: true, so /odom
        # ONLY publishes while a servo command is present. Hold center throughout.
        self.pub_servo = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)

        self.target_erpm = target_mps * gain * motor_sign
        self.spinup_s = spinup_s
        self.coast_timeout_s = coast_timeout_s
        self.target_mps = target_mps

        self.vel = []
        self.t = []
        self.t0 = None

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

    def _speed(self, erpm):
        self.pub_speed.publish(Float64(data=float(erpm)))

    def _current(self, amps):
        self.pub_current.publish(Float64(data=float(amps)))

    def _servo_center(self):
        # 0.55 = calibrated straight-ahead (from bridge mapping); keeps /odom alive
        self.pub_servo.publish(Float64(data=0.55))

    def run(self):
        # --- spin up to target in speed mode ---
        self.get_logger().info(f"Accelerating to {self.target_mps} m/s...")
        start = time.time()
        while time.time() - start < self.spinup_s:
            self._servo_center()
            self._speed(self.target_erpm)
            rclpy.spin_once(self, timeout_sec=0.02)

        # --- mark the coast start, then FREEWHEEL (0 torque) ---
        coast_start_idx = len(self.vel)
        coast_t0 = time.time() - self.t0 if self.t0 else 0.0
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
        if ct[i_lo] > 0:
            decel_magnitude = (cv[0] - cv[i_lo]) / ct[i_lo]
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
    args, ros_args = p.parse_known_args()

    rclpy.init(args=ros_args)
    node = CoastDownTest(args.target_mps, args.gain, args.motor_sign,
                         args.spinup, args.coast_timeout, args.odom_topic)

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
