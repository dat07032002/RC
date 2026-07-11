# RoboRacer Jetson — Session Handoff

Living handoff so a fresh agent/session can continue. Last updated 2026-07-09 after
Phase 1 floor testing and odometry calibration.

## Platform
- Jetson Orin Nano, JetPack 6.2 / L4T R36.4.7, Ubuntu 22.04, aarch64.
- ROS 2 **Humble**. `~/.bashrc` sources ROS + `~/f1tenth_ws` + `~/sim_ws`.
- sudo requires a password (the user has it; not stored here). Use `echo <pw> | sudo -S ...` when the user provides it.

## Remote access
- **Tailscale**: hostname `jetson-roboracer`, IP `100.91.57.60`, run with `--netfilter-mode=off` (L4T iptables quirk). Auto-starts.
- **SSH**: `ssh dat@jetson-roboracer` (or the 100.x IP). sshd enabled.
- **VNC** (full desktop, mirrors `:0`): `jetson-roboracer:5900`, password `roboracer` (x11vnc systemd service, enabled).
- Firefox default = **firefox-esr** (snap firefox is broken on L4T).

## Hardware
- Car: **Traxxas Ford Fiesta ST Rally VXL** (1/10). VESC replaces stock VXL-3s ESC.
- **VESC 60_MK6** (VESC 6 MkVI, HW60). USB `/dev/ttyACM0` → udev symlink `/dev/sensors/vesc`.
  - Firmware: user just flashed **stable 6.02** (was on dev 7.0 which had the servo bug).
  - FOC motor detection DONE: R 5.89 mΩ, L 2.57 µH, Ld-Lq 0.41 µH, λ 0.764 mWb, Sensorless.
- **LiDAR**: Hokuyo UST-10LX, Ethernet at `192.168.0.10:10940`. Jetson wired `enP8p1s0` = static `192.168.0.15/24` (NetworkManager profile `hokuyo-lidar`, persistent). Runs ~40 Hz.
- **IMU**: VESC onboard BMI160 → published on `/sensors/imu` via patched driver (~7.5 Hz).
- Bluetooth controller used today: 8BitDo Ultimate 2C Wireless via `/dev/input/js0`.
- No RealSense currently used.

## Software installed / built
- `~/f1tenth_ws` — f1tenth_system (all submodules: vesc, ackermann_mux, teleop_tools), + **particle_filter** (humble-devel) + range_libc, slam_toolbox, nav2. serial_driver installed.
- `~/sim_ws` — f1tenth_gym_ros (map path fixed to absolute). `~/f1tenth_gym` — f110_gym pip pkg.
- `~/vesc_tool/vesc_tool_7.00` — VESC Tool built from source (aarch64). Launch: `vesc_tool` or menu "VESC Tool".
- Driver patches applied in `~/f1tenth_ws` (also saved as patch, see below):
  - vesc_ackermann: `declare_parameter<T>()` + Float64 servo-sub fix (Humble compat).
  - **vesc_driver: added VESC IMU support** (COMM_GET_IMU_DATA → sensor_msgs/Imu on `/sensors/imu`).
  - f1tenth_stack `joy_teleop.yaml`: removed dead `default:` block that crashed joy_teleop.
  - Latent bug (not fixed, harmless): `if (driver_mode_ = MODE_OPERATING)` assignment in vesc_driver.cpp callbacks.

## The RC repo (github.com/dat07032002/RC)
- Branch: `phase1-vesc-bridge`.
- Files added/updated under `phase1_sysid/`: VESC bridge, sysid bringup, throttle/latency/odometry
  scripts, run guide, vehicle params, and VESC IMU patch notes.

## The PWM→VESC bridge (why it exists)
The Phase 1 scripts speak generic PWM (UInt16 1000-2000 on `/servo/command`, `/motor/command`).
`pwm_vesc_bridge.py` maps them to the VESC's Float64 `/commands/servo/position` and
`/commands/motor/speed` (eRPM, capped by `max_speed_mps`), with a 0.5 s motor watchdog.
Current verified mapping:
- Servo PWM 1000 -> `0.15` full left.
- Servo PWM 1500 -> `0.55` calibrated straight center.
- Servo PWM 2000 -> `0.85` full right.
- Motor sign is `-1.0`, because positive PWM originally drove the car physically backward.
`sysid_bringup.launch.py` runs ONLY vesc_driver + vesc_to_odom + urg + bridge (no joy/ackermann),
so scripts have exclusive control. Verified mapping + watchdog work.

## Phase 1 sysid progress (results in `phase1_sysid/config/vehicle_params.yaml`)
- **Vehicle geometry — DONE**: wheelbase `0.33 m`, track width `0.24 m`, mass estimate `3.6 kg`
  from component estimates until complete car can be weighed.
- **IMU — DONE**: gyro_bias [-0.000346,-0.008216,-0.005997] rad/s, gyro_noise 0.004072;
  accel_bias [0.2539,0.7187,10.4849] (Z reads ~7% high), accel_noise 0.0273.
- **LiDAR noise — DONE**: 1m σ0.0032, 2m σ0.0042, 5m σ0.0054; model σ=0.0029+0.00052·d;
  outliers ~0.3%. (10m skipped — unneeded indoors.) LiDAR freq corrected 25→40 Hz.
- **Steering — PARTIAL DONE**: wiring fixed by user; servo moves. Approx limits: left `30 deg`,
  right `25 deg`. Default `0.50` and trial `0.57` did not drive straight; `0.55` drove straight on
  the floor. Servo response delay/rate still not measured.
- **Throttle — PARTIAL DONE**: motor sign fixed; physical forward is now PWM 2000. Low-speed floor
  test with cap gave max velocity `0.57 m/s`, acceleration `0.61 m/s^2`. More space needed for
  coast-down friction and better max-speed curve.
- **Odometry — DONE enough for sim start**: saved factor `1.035` from the only run the user
  judged perfectly straight: `2.51 m / 2.424 m = 1.035`. Other 0.55-center runs
  (`2.42 m / 2.181 m = 1.110`, `2.42 m / 2.185 m = 1.108`) are documented but not used.
- **Latency — PARTIAL DONE**: corrected scan-to-odom pairing. 15 s sample: mean `11.33 ms`,
  median `7.60 ms`, min `0.02 ms`, max `29.97 ms`, LiDAR frequency `39.9 Hz`.
  Policy-to-motor and motor response latency remain open.

## 2026-07-11 controller + straight-line test
- **Controller pipeline fixed**: 8BitDo deadman is button index `6`, throttle is axis `1`,
  and full stick is software-capped at `2.0 m/s`.
- **Mux wiring fixed**: this ROS 2 `ackermann_mux` publishes `ackermann_drive_out`; the
  bringup now remaps it to `ackermann_cmd` for `ackermann_to_vesc_node`.
- **Steering validated straight**: teleop neutral now uses servo position `0.55`.
- **Test 7 complete**: 4 m release line, 4.25 m total physical travel, peak measured speed
  `2.12 m/s`, approximately 0.25 m stopping overshoot, and no VESC fault.
- **Odometry fixed and validated**: position propagation now uses VESC tachometer deltas
  rather than zero-order speed integration across irregular telemetry. Calibrated
  `speed_to_erpm_gain = 4529.41`; physical and reported distance agree. Wheelbase in the
  live VESC config corrected from `0.25` to measured `0.33 m`.
- Reproducible live-workspace changes are saved in
  `phase1_sysid/patches/controller_odom_2026-07-11.patch`.
- **Acceleration provisional**: accepted engineering estimate `3.5 m/s²`; use
  `2.5–4.5 m/s²` domain randomization. A hard launch produced substantial wheel slip, so
  wheel odometry could not provide chassis acceleration.
- **Turning radii measured**: full-left outer/inner diameters `1.66/1.18 m` give
  rear-axle radius `0.710 m` and `24.93°`; full-right `2.11/1.63 m` gives `0.935 m`
  and `19.44°`. Both diameter differences independently confirm the `0.24 m` track.
- **Asymmetric steering applied**: left/right servo gains are `-0.9194/-0.8842`
  servo units/radian. ROS command conversion, odometry, analytic simulation, PhysX task,
  USD joint limits, and viewer now preserve the measured asymmetry.

## Next tests
1. Coast-down friction with the dedicated zero-current/freewheel script and long runout.
2. Optional external-reference acceleration measurement to replace the estimate.
3. Optional improved servo video with command time-zero to replace the placeholder
   absolute response delay.

## Operational gotchas
- Only ONE of {VESC Tool, ROS stack} may hold `/dev/ttyACM0` at a time.
- Only ONE sysid launch at a time — an orphaned `urg_node` keeps the LiDAR's single TCP slot
  (`:10940`) and blocks the next launch. Clean everything:
  `pkill -9 -f "vesc_driver_node|urg_node_driver|pwm_vesc_bridge|vesc_to_odom_node"`
- `setsid`/background launches orphan easily; verify with `ps` and kill by PID if needed.
- `ros2 node list` can show stale nodes; `ros2 daemon stop && ros2 daemon start` clears cache.
- **User prefers to run on-car/hardware tests themselves** — prepare + interpret, don't execute
  motion tests unless asked. Read-only probes are fine.
