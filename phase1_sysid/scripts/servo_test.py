#!/usr/bin/env python3
"""
Servo Response Testing for Roboracer Phase 1
Measures steering gain, response delay, and max angles
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16
import time
import sys

class ServoTest(Node):
    def __init__(self):
        super().__init__('servo_test')
        self.pub = self.create_publisher(UInt16, '/servo/command', 10)
        self.get_logger().info("=" * 60)
        self.get_logger().info("SERVO RESPONSE TEST")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Commands:")
        self.get_logger().info("  'left'   - Full left (PWM 1000)")
        self.get_logger().info("  'center' - Center (PWM 1500)")
        self.get_logger().info("  'right'  - Full right (PWM 2000)")
        self.get_logger().info("  'sweep'  - Sweep left to right")
        self.get_logger().info("  'test'   - Step response test")
        self.get_logger().info("  'exit'   - Quit")
        self.get_logger().info("=" * 60)
        self.get_logger().info("\nRECORD STEERING ANGLES with phone level app!")
        self.get_logger().info("To measure response delay: use slow-motion video (120fps)\n")

    def send_pwm(self, pwm, name=""):
        """Send PWM command to servo."""
        msg = UInt16(data=pwm)
        self.pub.publish(msg)
        self.get_logger().info(f"→ PWM {pwm:4d} µs {name}")

    def run_test_sweep(self):
        """Sweep steering from left to right."""
        self.get_logger().info("\n--- SWEEP TEST: Left → Center → Right ---")
        self.get_logger().info("Recording angles with phone level app at each position:")

        positions = [
            (1000, "LEFT FULL"),
            (1200, "LEFT 3/4"),
            (1350, "LEFT 1/4"),
            (1500, "CENTER"),
            (1650, "RIGHT 1/4"),
            (1800, "RIGHT 3/4"),
            (2000, "RIGHT FULL"),
        ]

        for pwm, label in positions:
            self.send_pwm(pwm, f"({label})")
            print(f"\n  PAUSE: Measure steering angle at {label}")
            print(f"  Use phone level app app, record: _____ degrees")
            input("  Press Enter when angle measured... ")

    def run_step_response_test(self):
        """Full-range step for slow-mo video analysis (lag + slew rate).

        Keep the TERMINAL in the camera frame. The big banner flashes on the
        exact frame the command is sent, giving a video time-zero reference.
        Do one direction per clip (e.g. center->right), then re-run for left.
        """
        self.get_logger().info("\n--- STEP RESPONSE TEST (full range) ---")
        direction = input("Step direction (right/left) [right]: ").strip().lower()
        target_pwm = 1000 if direction == "left" else 2000
        target_name = "LEFT FULL" if direction == "left" else "RIGHT FULL"

        print("\nCAR ON STAND, wheels off ground.")
        print("Start slow-mo video NOW (120/240fps). Keep this terminal IN FRAME.")
        input("Press Enter, then a 3s countdown starts...")

        # Settle at center so the wheel is visibly still before the step.
        self.send_pwm(1500, "(center, settling)")
        time.sleep(1.0)
        for n in (3, 2, 1):
            print(f"   ... {n}")
            time.sleep(1.0)

        # Fire the step and stamp it. The banner + monotonic time land in the
        # same instant the PWM is published; that frame is video t=0.
        t0 = time.monotonic()
        self.send_pwm(target_pwm, f"({target_name})")
        print("\n" + "#" * 50)
        print(f"###  COMMAND SENT  t0={t0:.3f}s  -> {target_name}  ###")
        print("#" * 50 + "\n")
        time.sleep(1.5)  # hold so the wheel fully settles on camera

        self.send_pwm(1500, "(return to center)")
        print("Step done. Stop the video.")
        print("Send me the video path; I'll extract frames and compute")
        print("lag (ms) and slew rate (deg/s). Re-run for the other direction.")

    def run_interactive(self):
        """Interactive servo control."""
        while True:
            cmd = input("\nCommand (left/center/right/sweep/test/exit): ").strip().lower()

            if cmd == "left":
                self.send_pwm(1000, "(LEFT FULL)")
                angle = input("Measured angle (degrees): ")
                print(f"→ Record: angle_left_max = {angle}")

            elif cmd == "center":
                self.send_pwm(1500, "(CENTER)")
                angle = input("Measured angle (degrees, should be ~0): ")
                print(f"→ Record: angle_center = {angle}")

            elif cmd == "right":
                self.send_pwm(2000, "(RIGHT FULL)")
                angle = input("Measured angle (degrees): ")
                print(f"→ Record: angle_right_max = {angle}")

            elif cmd == "sweep":
                self.run_test_sweep()

            elif cmd == "test":
                self.run_step_response_test()

            elif cmd == "exit":
                self.get_logger().info("Exiting servo test")
                break

            else:
                self.get_logger().warning("Invalid command")

            time.sleep(0.2)

if __name__ == '__main__':
    rclpy.init()
    node = ServoTest()

    try:
        node.run_interactive()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
