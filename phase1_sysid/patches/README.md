# vesc_driver patch (IMU + Humble compat)

`vesc_driver_imu.patch` applies to the **f1tenth/vesc** repo (the `vesc`
submodule of `f1tenth_system`). It contains the local changes needed on this
Jetson:

1. **IMU support** — request `COMM_GET_IMU_DATA` and publish `sensor_msgs/Imu`
   on `/sensors/imu` from the VESC 6 MkVI's onboard IMU (accel in m/s²,
   angular rate in rad/s, orientation quaternion when AHRS is enabled).
2. **Humble build fixes** — `declare_parameter<T>()` template form, and the
   servo subscription type fix in `vesc_to_odom`.

## Apply on a fresh checkout
```bash
cd ~/f1tenth_ws/src/f1tenth_system/vesc
git apply ~/RC/phase1_sysid/patches/vesc_driver_imu.patch
git apply ~/RC/phase1_sysid/patches/vesc_to_odom_publish_tf.patch
cd ~/f1tenth_ws
colcon build --symlink-install --packages-select vesc_driver vesc_ackermann
```

Verify: `ros2 topic echo /sensors/imu` should show gravity (~9.8 m/s²) on one
accel axis while the car is level and still.
