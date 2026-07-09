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
| 1000 | 0.15 (full left) | −9228 eRPM (rev, capped) |
| 1500 | 0.50 (center) | 0 (stop) |
| 2000 | 0.85 (full right) | +9228 eRPM (= 2.0 m/s cap) |

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
    --ros-args -r /odometry/filtered:=/odom
```
Start with `max_speed_mps:=1.0` in the launch until you trust it, then raise.
Measures `max_velocity`, `max_acceleration`, coast-down friction.

## 4. Latency  — remap odom too
```bash
python3 ~/RC/phase1_sysid/scripts/latency_test.py \
    --ros-args -r /odometry/filtered:=/odom
```

## 5. IMU calibration  — ⛔ BLOCKED
The installed `vesc_driver` does not publish the VESC's onboard IMU
(no `COMM_GET_IMU_DATA` support), and no RealSense is connected.
Options: (a) add IMU support to `vesc_driver`, or (b) add a RealSense/dedicated IMU.

## Record everything
```bash
ros2 bag record -a -o sysid_$(date +%H%M)
```

Fill measured values into `config/vehicle_params.yaml`.
