# Session continuation — 2026-07-11

This file summarizes work completed after the verbatim transcript in
`SESSION_TRANSCRIPT_2026-07-11.txt`.

## Controller and straight-line test

- Diagnosed a ROS topic disconnect: `ackermann_mux` published
  `/ackermann_drive_out` while `ackermann_to_vesc_node` subscribed to
  `/ackermann_cmd`. Corrected the launch remapping.
- Mapped the 8BitDo Ultimate 2C: deadman button index 6, throttle axis 1.
- Reduced full-stick indoor speed from 5 m/s to 2 m/s.
- Corrected straight servo center from 0.5304 to the floor-validated 0.55.
- Completed the controller run: 4 m release line, 4.25 m total travel, peak
  wheel speed 2.12 m/s, approximately 0.25 m stopping overshoot, no VESC fault.

## Odometry

- Found that zero-order integration applied a late high-speed sample across a
  long telemetry gap, producing a false position jump.
- Replaced speed integration with signed VESC tachometer deltas. VESC counts
  persist through missing samples, so distance remains correct.
- Calibrated `speed_to_erpm_gain=4529.41` from 1925 counts over 4.25 m.
- Corrected wheelbase from 0.25 m to the measured 0.33 m.
- User validated that physical and reported straight-line distance agree.

## Acceleration

- A hard launch produced 4.42 m equivalent wheel rotation over only 2.43 m
  chassis travel, demonstrating substantial slip. Cumulative odometry was
  explicitly removed by subtracting pre-run from post-run pose; the mismatch
  was also independently present in the raw tachometer delta.
- Wheel odometry therefore cannot measure chassis acceleration during this
  launch. No authoritative manufacturer acceleration specification exists.
- Accepted provisional engineering estimate: 3.5 m/s², with simulation domain
  randomization from 2.5 to 4.5 m/s².

## Turning geometry

- Left full lock: outer/inner rear-wheel diameters 1.66/1.18 m, rear-axle-center
  radius 0.710 m, effective angle 24.93°.
- Right full lock: outer/inner diameters 2.11/1.63 m, radius 0.935 m, effective
  angle 19.44°.
- Both diameter differences were 0.48 m, independently confirming the 0.24 m
  track width.
- Implemented direction-specific servo gains: left −0.9194, right −0.8842.
- Updated ROS command conversion, odometry, analytic simulation, PhysX task,
  USD limits, viewer, vehicle parameters, and system design.

## IMU Plan A result

- The intended architecture requires an approximately 100 Hz IMU for the EKF.
- VESC BMI160 requests were reduced from all 16 fields to only accel+gyro using
  protocol mask `0x01F8`.
- Aggressive IMU priority reached approximately 44 Hz.
- A balanced test reached approximately 38.6 Hz IMU but collapsed VESC motor
  state to approximately 3.7 Hz with gaps up to 1.18 s.
- Restored motor-priority polling. A clean final check measured approximately
  8.7 Hz VESC state and 6.0 Hz BMI160, with a worst gap near 1.12 s.
- Plan A is rejected. The in-hand SparkFun BNO086 over Jetson I²C is selected
  as primary production IMU; BMI160 remains backup/diagnostics.

## Remaining work

1. Install, configure, calibrate, and dynamically validate the BNO086 at ≥100 Hz.
2. Run zero-current freewheel coast-down to identify rolling/tire/drag behavior.
3. Optionally replace provisional acceleration with external video measurement.
4. Optionally re-film servo motion with visible command time-zero.
5. Weigh the complete ready-to-drive car to replace the 3.6 kg estimate.
