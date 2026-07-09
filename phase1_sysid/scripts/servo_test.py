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
        """Measure response time with video."""
        self.get_logger().info("\n--- STEP RESPONSE TEST ---")
        self.get_logger().info("1. Start recording slow-motion video (120fps preferred)")
        input("Press Enter to start step input...")

        self.get_logger().info("Sending step: 1500 µs → 1700 µs")
        self.send_pwm(1500, "(initial)")
        time.sleep(0.5)

        # Record: step up
        print("\n>>> VIDEO RECORDING <<<")
        self.send_pwm(1700, "(STEP UP)")
        time.sleep(1.0)

        # Return to center
        self.send_pwm(1500, "(return to center)")

        self.get_logger().info("\n2. Analyze video:")
        self.get_logger().info("   - Count frames from command to 90% of final position")
        self.get_logger().info("   - Delay (ms) = (frame_count / fps) * 1000")
        response_time = input("Enter response time (ms): ")

        try:
            self.get_logger().info(f"Measured response delay: {response_time} ms")
            print(f"→ Record in vehicle_params.yaml: servo_response_delay = {response_time}")
        except:
            pass

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
