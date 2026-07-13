# VESC source patches

`vesc_mapping_stack.patch` is the consolidated patch for a fresh
`f1tenth/vesc` checkout. It includes:

1. ROS 2 Humble parameter and message-type compatibility fixes.
2. Direction-specific steering calibration support.
3. Tachometer-delta wheel odometry and correct TF parameter loading.
4. VESC onboard IMU decoding for optional diagnostics.
5. State-only 20 Hz polling with `COMM_GET_VALUES_SELECTIVE`, requesting only
   RPM, voltage, tachometers, and fault status. The production BNO086 supplies
   the mapping IMU.

The older individual patch files are retained as development history. Do not
stack them with the consolidated patch on a fresh checkout.

## Apply on a fresh checkout

```bash
cd ~/f1tenth_ws/src/f1tenth_system/vesc
git apply ~/RC/phase1_sysid/patches/vesc_mapping_stack.patch
cd ~/f1tenth_ws
colcon build --symlink-install --packages-select vesc_driver vesc_ackermann
```

## Verify

With the car stationary and a zero-speed command active:

```bash
ros2 topic hz /sensors/core
ros2 topic hz /wheel/odom
ros2 topic hz /odom
```

Targets are about 20 Hz raw VESC/wheel odometry and 30 Hz EKF odometry.
