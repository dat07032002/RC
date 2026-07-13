# BNO086-assisted higher-speed coast-down — 2026-07-13

- Commanded coast-start speed: 1.2 m/s
- Integrated coast-start speed: 1.046 m/s
- Delayed VESC-odometry peak: 0.852 m/s
- BNO086 sample rate: approximately 100.5 Hz
- Automatically detected vehicle-forward IMU axis: +X
- Stationary baseline: `[-0.0616, -0.2884, 9.6620] m/s²`
- Integrated velocity loss: 1.046 m/s
- Time to minimum integrated velocity: 1.662 s
- Mean acceleration to that point: -0.629 m/s²
- Median acceleration from 0.1 s to 1.662 s: -0.610 m/s²
- Effective resistance ratio, `|a| / g`: 0.064

The lower-speed BNO086 run measured -0.611 m/s². The 3% increase at the
higher speed is within run-to-run variation, so rolling and driveline resistance
dominate over aerodynamic drag in the tested range. The combined nominal value
is -0.620 m/s²; use `[-0.50, -0.75] m/s²` for simulation randomization.
