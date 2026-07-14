# Improved school SLAM run — 2026-07-14

## Outcome

The sensor and odometry timing improvements passed, but the occupancy map
**failed visual quality validation**. The map still contains duplicated wall
outlines and radial scan streaks, especially on the right and lower sides.
Do not use this map for localization.

## Saved Jetson artifacts

- `/home/dat/maps/school_room_improved_2026-07-14.pgm`
- `/home/dat/maps/school_room_improved_2026-07-14.yaml`
- `/home/dat/maps/school_room_improved_2026-07-14.posegraph`
- `/home/dat/maps/school_room_improved_2026-07-14.data`
- `/home/dat/bags/school_room_mapping_improved_2026-07-14/`

## Run summary

- Duration: 444.03 s
- Bag size: 121.2 MiB
- Recorded messages: 149,389
- Map dimensions: 310 x 312 pixels at 0.05 m/pixel
- Driven odometry distance: 36.76 m
- Maximum speed: 0.733 m/s
- 95th-percentile speed: 0.496 m/s

## Timing and motion quality

| Measurement | Result |
|---|---:|
| LiDAR receive interval p95 / max | 0.030 / 0.050 s |
| EKF odometry interval p95 / max | 0.034 / 0.067 s |
| EKF translation step p95 / max | 0.017 / 0.037 m |
| EKF yaw step p95 / max | 0.015 / 0.035 rad |
| BNO gyro-Z vs wheel-yaw correlation | 0.991 |

The failed 2026-07-13 run had odometry gaps up to 3.29 s, translation jumps up
to 1.68 m, and yaw jumps up to 1.80 rad. Those timing failures are resolved.

## Interpretation

Because timing, filtered odometry continuity, and yaw-rate agreement now pass,
the remaining map corruption is more likely caused by scan geometry or
calibration: LiDAR tilt/vibration, an inaccurate LiDAR yaw transform, or EKF
yaw weighting/IMU mounting alignment during turns.

## Next diagnostic

1. Perform a stationary LiDAR scan-plane and rigid-mount check.
2. Record a short straight-line pass alongside one flat wall.
3. Record one slow 360-degree rotation in place or the smallest safe circle.
4. Compare LiDAR wall alignment against BNO-integrated yaw and EKF yaw before
   attempting another full-room map.
