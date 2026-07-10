#!/usr/bin/env python3
"""
PWM -> VESC bridge for Phase 1 System ID.

The Phase 1 sysid scripts speak a generic PWM interface (UInt16, 1000-2000 us):
    /servo/command   (1000=left, 1500=center, 2000=right)
    /motor/command   (1500=stop, 2000=full fwd, 1000=full rev)

The installed f1tenth_system drives the car through the VESC, which expects:
    /commands/servo/position   std_msgs/Float64  in [0.0, 1.0]
    /commands/motor/speed      std_msgs/Float64  in eRPM

This node translates between them so the sysid scripts run unchanged.

SAFETY:
  * A watchdog stops the motor (0 eRPM) if no /motor/command arrives within
    `cmd_timeout` seconds. Steering is held.
  * Motor speed is hard-capped at `max_speed_mps` (converted to eRPM).
  * Publishing 1500 on /motor/command always maps to exactly 0 eRPM.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16, Float64


class PwmVescBridge(Node):
    def __init__(self):
        super().__init__('pwm_vesc_bridge')

        # --- parameters (override on the command line if needed) ---
        self.declare_parameter('speed_to_erpm_gain', 4614.0)   # from vesc.yaml
        self.declare_parameter('max_speed_mps', 2.0)           # SAFETY cap
        self.declare_parameter('motor_sign', -1.0)             # +PWM was physically reverse
        self.declare_parameter('servo_min', 0.15)              # full left
        self.declare_parameter('servo_center', 0.55)           # calibrated straight-ahead
        self.declare_parameter('servo_max', 0.85)              # full right
        self.declare_parameter('cmd_timeout', 0.5)             # motor watchdog (s)

        self.k_erpm   = self.get_parameter('speed_to_erpm_gain').value
        self.max_mps  = self.get_parameter('max_speed_mps').value
        self.motor_sign = self.get_parameter('motor_sign').value
        self.servo_lo = self.get_parameter('servo_min').value
        self.servo_mid = self.get_parameter('servo_center').value
        self.servo_hi = self.get_parameter('servo_max').value
        self.timeout  = self.get_parameter('cmd_timeout').value

        self.max_erpm = self.max_mps * self.k_erpm

        # VESC command publishers (vesc_driver_node subscribes to these)
        self.pub_servo = self.create_publisher(Float64, '/commands/servo/position', 10)
        self.pub_speed = self.create_publisher(Float64, '/commands/motor/speed', 10)

        # PWM command inputs (from the sysid scripts)
        self.create_subscription(UInt16, '/servo/command', self.on_servo, 10)
        self.create_subscription(UInt16, '/motor/command', self.on_motor, 10)

        self.last_motor_cmd = self.get_clock().now()
        self.create_timer(0.1, self.watchdog)   # 10 Hz safety check

        self.get_logger().info(
            f"PWM->VESC bridge up. max_speed={self.max_mps} m/s "
            f"(={self.max_erpm:.0f} eRPM cap), "
            f"servo=[{self.servo_lo},{self.servo_mid},{self.servo_hi}], "
            f"motor_sign={self.motor_sign}, "
            f"motor watchdog={self.timeout}s")

    @staticmethod
    def _clamp(x, lo, hi):
        return max(lo, min(hi, x))

    def on_servo(self, msg):
        # Piecewise map so PWM 1500 holds calibrated straight-ahead steering.
        pwm = self._clamp(msg.data, 1000, 2000)
        if pwm <= 1500:
            frac = (pwm - 1000) / 500.0
            pos = self.servo_lo + frac * (self.servo_mid - self.servo_lo)
        else:
            frac = (pwm - 1500) / 500.0
            pos = self.servo_mid + frac * (self.servo_hi - self.servo_mid)
        self.pub_servo.publish(Float64(data=pos))

    def on_motor(self, msg):
        self.last_motor_cmd = self.get_clock().now()
        # 1500 -> 0 eRPM; motor_sign maps positive PWM to physical forward.
        frac = self._clamp((msg.data - 1500) / 500.0, -1.0, 1.0)
        erpm = frac * self.max_erpm * self.motor_sign
        self.pub_speed.publish(Float64(data=erpm))

    def watchdog(self):
        dt = (self.get_clock().now() - self.last_motor_cmd).nanoseconds / 1e9
        if dt > self.timeout:
            # stale -> command a stop (cheap, idempotent)
            self.pub_speed.publish(Float64(data=0.0))


def main():
    rclpy.init()
    node = PwmVescBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # stop motor on exit, but only if the context is still valid
        if rclpy.ok():
            try:
                node.pub_speed.publish(Float64(data=0.0))
            except Exception:
                pass
            rclpy.shutdown()


if __name__ == '__main__':
    main()
