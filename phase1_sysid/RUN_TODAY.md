# Phase 1 System ID — Run Guide (VESC stack)

The original scripts assume a generic PWM interface. This car uses a **VESC 6 MkVI**,
so a bridge node translates PWM (1000–2000 µs) to VESC commands.

## 0. Start the sysid stack (every session)

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_ws/install/setup.bash
ros2 launch ~/RC/phase1_sysid/config/sysid_bringup.launch.py max_speed_mps:=2.0
```

This starts **only**: `vesc_driver`, `vesc_to_odom` (`/odom`), `urg_node` (`/scan`),
and `pwm_vesc_bridge`. No joystick/ackermann pipeline, so the scripts have exclusive
control of the VESC. Leave this running in one terminal; run scripts in another.

Bridge mapping (verified):
| PWM | Servo (`/commands/servo/position`) | Motor (`/commands/motor/speed`) |
|-----|-----|-----|
| 1000 | 0.15 (full left) | +9228 eRPM (physical reverse, capped) |
| 1500 | 0.55 (calibrated straight center) | 0 (stop) |
| 2000 | 0.85 (full right) | −9228 eRPM (physical forward, capped) |

Safety: motor **watchdog** commands 0 eRPM if no `/motor/command` for 0.5 s.
Raise/lower the speed cap with `max_speed_mps:=`.

## 1. LiDAR noise  ✅ works as-is (no motion)
```bash
python3 ~/RC/phase1_sysid/scripts/lidar_noise_test.py
```
Point the car at a flat wall; measure at 1 / 2 / 5 / 10 m with a tape measure.
(Validated: at ~0.6 m the sensor showed σ ≈ 0.009 m — sane for a UST-10LX.)

## 2. Servo characterization  — CAR ON A STAND (wheels off ground)
```bash
python3 ~/RC/phase1_sysid/scripts/servo_test.py
```
Use `left/center/right/sweep`, measure wheel angle with a phone level app.
Record `angle_left_max`, `angle_center` (should be ~0 — if not, tune
`steering_angle_to_servo_offset` in `vesc.yaml`), `angle_right_max`.

## 3. Throttle / velocity  — NEEDS ~10 m CLEAR FLOOR
Odom topic differs, so remap `/odometry/filtered` → `/odom`:
```bash
python3 ~/RC/phase1_sysid/scripts/throttle_test.py \
    --odom-topic /odom
```
Start with `max_speed_mps:=1.0` in the launch until you trust it, then raise.
Measures `max_velocity`, `max_acceleration`, coast-down friction.

For the available ~6 m floor, use the short capped test:
```bash
python3 ~/RC/phase1_sysid/scripts/throttle_test.py \
    --levels 100 --duration 1.0 --skip-coast-down --odom-topic /odom
```

## 4. Latency  — remap odom too
```bash
python3 ~/RC/phase1_sysid/scripts/latency_test.py \
    --duration 15 --odom-topic /odom
```

## 5. Odometry distance calibration — fits ~6 m floor
Start with the low speed cap, e.g. `max_speed_mps:=0.3`, then run:
```bash
python3 ~/RC/phase1_sysid/scripts/odometry_test.py \
    --duration 8 --odom-topic /odom
```
Measure the physical start-to-stop distance on the floor and compute:
`distance_correction_factor = physical_distance_m / odom_distance_m`.
Saved calibration: center `0.55`, trusted straight run `2.51 / 2.424` -> correction `1.035`.
The `2.42 / 2.181` and `2.42 / 2.185` runs are documented but not used because the car
was not as straight.

## Today's measured values
- Vehicle geometry: wheelbase `0.33 m`, track width `0.24 m`, mass estimate `3.6 kg`.
- Steering: servo command center `0.55`; left max about `30 deg`, right max about `25 deg`.
- Throttle: at low cap, `max_velocity 0.57 m/s`, `max_acceleration 0.61 m/s^2`.
- Odometry: `distance_correction_factor 1.035`, `velocity_correction_factor 1.035`.
- Latency: LiDAR-to-odom mean about `11 ms`; LiDAR frequency about `40 Hz`.
- Remaining: servo response delay/rate, coast-down friction, LiDAR mount offsets, better mass.

## 6. IMU calibration  — ✅ now works (patched vesc_driver)
`vesc_driver` was patched to request `COMM_GET_IMU_DATA` and publish
`sensor_msgs/Imu` on `/sensors/imu` (~7.5 Hz) from the MkVI's onboard IMU.
The script listens on `/imu`, so remap:
```bash
python3 ~/RC/phase1_sysid/scripts/imu_calib.py \
    --ros-args -r /imu:=/sensors/imu
```
Keep the car **stationary and level** for ~1 minute. Records gyro bias,
accel bias (Z should be ~9.81; measured ~10.6 raw -> needs the bias/scale
this test produces), and noise std.

## Record everything
```bash
ros2 bag record -a -o sysid_$(date +%H%M)
```

Fill measured values into `config/vehicle_params.yaml`.
