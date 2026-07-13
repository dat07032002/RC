# BNO086-assisted coast-down repeat — 2026-07-13

- Commanded coast-start speed: 0.8 m/s
- BNO086 sample rate: approximately 103 Hz
- Automatically detected vehicle-forward IMU axis: +X
- Stationary baseline: `[-0.0495, -0.4873, 9.6538] m/s²`
- Integrated velocity loss: 0.777 m/s
- Time to minimum integrated velocity: 1.272 s
- Mean acceleration to that point: -0.611 m/s²
- Median acceleration from 0.1 s to 1.272 s: -0.591 m/s²
- Effective resistance ratio, `|a| / g`: 0.062

The VESC odometry stream remained stale at zero during this run. The BNO086 curve
is the primary result; the earlier three-point odometry fit of -0.752 m/s² is a
coarse independent cross-check. Use `[-0.50, -0.75] m/s²` for simulation domain
randomization around the measured value.
