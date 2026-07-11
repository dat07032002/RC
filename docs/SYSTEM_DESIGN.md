# RoboRacer System Design (Hardware-Fitted Revision)

Revised 2026-07-09. Architecture: Sensors → State Estimation → RL Policy →
Safety Layer → Low-Level Control, re-specified against the real platform.
(M) = measured, (P) = provisional/single-run, (?) = unmeasured.

```
UST-10LX 40Hz + VESC IMU/odom
        ↓
EKF (fast, local) + particle filter vs SLAM map (slow, global)
        ↓
PPO+GRU policy @ 20 Hz  (goal-conditioned, uncertainty-aware)
        ↓
Safety supervisor @ 40 Hz (raw scan, stopping-distance + arc check)
        ↓
Slew limiter + VESC bridge (VESC closes speed/servo loops internally)
```

## 0. Platform constraints

| Constraint | Reality | Consequence |
|---|---|---|
| Compute | Orin Nano shared: SLAM + EKF + policy + safety | policy 20 Hz |
| LiDAR | 40 Hz, 270°, 10 m guaranteed (M) | perception ceiling 25 ms |
| IMU | BNO086 on Jetson I²C (selected); VESC BMI160 backup | Plan A bandwidth test failed |
| Wheel encoder | VESC eRPM telemetry is the encoder | odom scale 1.035 (P, n=1) |
| Steering feedback | none (hobby servo) | steering state modeled, never measured |
| Speed | 0.57 m/s capped test (M); true max (?) | design speed ≤2 m/s until tire model |
| Brake decel | (?) | safety uses conservative 1.5 m/s² placeholder |
| GPS | none (indoor) | "GPS denial" → localization degradation |
| Camera | RealSense shelved | LiDAR-only for now |

## 1. Sensors

- **LiDAR 40 Hz**: 1081 beams → 64 for policy; safety layer uses full raw scan.
- **IMU — Plan B selected after measured Plan A failure**: The VESC BMI160
  packet was reduced to accel+gyro only and scheduled aggressively. IMU reached
  only ~44 Hz alone or ~38.6 Hz in a balanced test, while motor state collapsed
  to ~3.7 Hz with gaps up to 1.18 s. The request/response path cannot meet the
  ~100 Hz EKF requirement without damaging wheel telemetry. Use the **SparkFun
  BNO086 on Jetson I²C** as the primary IMU; retain BMI160 as diagnostics/backup.
- **Wheel odometry**: VESC eRPM ≥50 Hz, ×1.035 correction (P — re-verify).
- **Steering feedback: deleted.** First-order model δ̇=(δ_cmd−δ)/τ, τ from
  pending stand test; safety margins absorb model error.
- **GPS: deleted.**

## 2. State estimation

- Fast/local: EKF (`robot_localization`) fusing IMU (~100 Hz) + wheel odom
  (50 Hz) → smooth [v, ψ̇], dead-reckoned pose between scans.
- Slow/global: `slam_toolbox` maps; `particle_filter` corrects pose at 40 Hz.
  Particle spread = uncertainty output for policy + safety.
- State: [x, y, ψ, v, ψ̇] + covariance.
- Latency: v1 = timestamp-aware fusion + constant-velocity prediction to
  actuation time. Full state-replay buffer only if measured end-to-end
  latency exceeds ~60 ms.
- Localization denial: PF confidence collapses → EKF dead-reckons →
  uncertainty grows → policy slows (learned) + safety shrinks envelope
  (guaranteed). Goal stays valid in map frame.

## 3. RL policy

- **PPO + GRU** first (parallel Isaac + 5×RTX 6000 → sample efficiency is
  not the constraint; recurrent SAC/TQC is fallback; Dreamer stays on the
  research roadmap).
- **20 Hz**: matches sim decimation (100 Hz physics / 5) exactly; fits Orin
  budget; servo can't follow faster commands anyway.
- Obs: 64 LiDAR / v / modeled δ / prev action / **final-goal** bearing+distance
  / pose-uncertainty scalar / latency estimate.
  Goal is the FINAL destination in the room frame, not a route waypoint: the
  physical testbed (fixed start/end points, track shape AND obstacles change
  between runs) means no reliable route/map exists at deployment. The policy
  follows the track from LiDAR; goal direction is guidance only.
- Act: [v_target ≤ 2 m/s indoor cap, δ_target ∈ −19.44° right / +24.93° left].

## 4. Safety layer (build + trust FIRST)

Standalone node, no RL deps, override authority ahead of the bridge.
1. d_stop = v·τ_total + v²/(2·a_brake) + d_margin; τ_total 100 ms
   conservative, a_brake 1.5 m/s² (? — measure), d_margin 0.3 m; vs min raw
   scan in the steering-feasible cone.
2. Arc-existence check within the asymmetric measured steering limits + steer-rate limit;
   no free arc → stop.
3. Envelope governor: speed ceiling shrinks with uncertainty, proximity,
   scan staleness.
4. Watchdogs: bridge 0.5 s motor watchdog (exists) + scan-age >100 ms →
   stop + estimator-health.
Rate 40 Hz scan-triggered + 10 Hz timer (no new obstacle info between
scans). Always newest raw scan, never the SLAM map.

## 5. Low-level control

MPC deleted for v1. VESC firmware closes speed (FOC/eRPM PID, ~kHz) and the
servo is open-loop by nature. Our layer = unit mapping (servo 0.15/0.55/0.85,
eRPM gain) + accel/steer slew limits (from measured a_max, servo test) +
safety override input. Evolution of `pwm_vesc_bridge.py`. MPC/pure-pursuit
returns only for friction-limit racing (post tire model).

## 6. Rate table

| Loop | Original | Revised |
|---|---|---|
| LiDAR | 40 | 40 (M) |
| IMU/EKF | 200–400 | ~100 (fix pending) |
| Particle filter | — | 40 |
| RL policy | 40–50 | 20 |
| Safety | 100–200 | 40 + watchdogs |
| Speed/steer | 100–200 | VESC internal kHz + 50 Hz setpoints |

## 7. Training environment (phase3_sim/roboracer_isaaclab_task.py)

DR anchored to measurements: corridor 0.5–2.0 m · turns 3–8 · obstacles
static→appearing→moving · dead ends/blockages · friction wide · LiDAR noise
(a,b)×[0.5,5] + beam dropout · **odom scale ∈ [1.00, 1.12] (measured spread)**
· action delay 20–150 ms · v/a ±30%.

Curriculum: 1 fixed track ✅ · 2 random tracks ✅ · 3 goal + static obstacles
· 4 appearing/moving obstacles · 5 mid-episode geometry change · 6 blocked
routes → correct stop · 7 sensor faults + localization dropout · 8 real car
from 0.5 m/s.

## 8. Reward

Progress + safe speed + goal + smoothness − collision − boundary − unsafe
speed − excess steering. Stop semantics: rewarded iff no collision-free path
exists (sim ground truth); idling below creep speed >2 s otherwise penalized.

## 9. Open measurements the design depends on

1. Servo τ + delay (stand test) → steering model, safety arcs
2. Brake/coast decel → a_brake (currently guessed 1.5)
3. True v_max / throttle curve → action scaling
4. Policy→motor latency on Orin → τ_total
5. Install/configure BNO086 on Jetson I²C and validate ≥100 Hz → EKF viability
6. LiDAR mount offset → TF tree, map quality
7. Odometry factor repeatability (n=1) → DR confidence
