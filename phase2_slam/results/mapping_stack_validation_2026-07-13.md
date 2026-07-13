# Improved mapping stack validation — 2026-07-13

## Changes

- Replaced full VESC state and onboard-IMU polling with a 20 Hz selective
  telemetry request containing RPM, voltage, tachometers, and fault status.
- Added `robot_localization` EKF at 30 Hz.
- Fused raw wheel forward speed/yaw rate with BNO086 gyro Z.
- Made the EKF the only `odom -> base_link` TF publisher.
- Reduced SLAM scan processing from 40 Hz to about 10 Hz.
- Set the UST-10LX maximum mapping range to 10 m.

## Stationary 30-second gate

| Topic | Rate | p95 gap | p99 gap | Maximum gap |
|---|---:|---:|---:|---:|
| `/sensors/core` | 18.89 Hz | 0.052 s | 0.152 s | 0.446 s |
| `/wheel/odom` | 18.89 Hz | 0.052 s | 0.153 s | 0.445 s |
| `/odom` (EKF) | 30.00 Hz | 0.035 s | 0.035 s | 0.036 s |
| `/scan` | 39.92 Hz | 0.031 s | 0.032 s | 0.034 s |
| `/imu/data` | 99.76 Hz | 0.017 s | 0.018 s | 0.043 s |

The EKF maintained uninterrupted 30 Hz odometry through the one raw VESC
serial gap. The failed mapping run had odometry gaps up to 3.29 s, so the
filtered transform gap improved by about two orders of magnitude.

## IMU axis validation

The failed-run bag supplied 170 moving yaw-rate comparisons. Correlation with
wheel yaw rate was 0.057 for gyro X, 0.313 for gyro Y, and 0.863 for gyro Z.
Gyro Z also had the same sign as wheel yaw, supporting the EKF configuration.

## Remaining gate

Run a short low-speed dynamic map at 0.4-0.6 m/s. Accept it only if walls stay
single during turns and remain aligned after returning to the start marker.
