# Expected Measurement Ranges - Sanity Check Guide

**Know what "good" looks like before testing**

---

## 🎯 Steering Servo

### Expected Ranges
| Measurement | Good | Acceptable | Bad |
|---|---|---|---|
| **Left max angle** | 30°–40° | 25°–45° | <20° or >50° |
| **Right max angle** | 30°–40° | 25°–45° | <20° or >50° |
| **Response delay** | 50–100ms | 30–150ms | >200ms |
| **Min steering** | 5°–10° | 1°–20° | >30° |

### Why These Ranges?
- **Steering angle 30-40°:** Standard RC servo range; tight corners need 30°+
- **Response delay <100ms:** Real-time control; >200ms = sluggish, oscillatory
- **Good servo:** Smooth movement, no jitter, full range accessible

### If Measurements Are Off
| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Angles <20° | Limited servo throw | Check servo calibration, mechanical limits |
| Angles >50° | Over-calibrated | Reduce PWM range (1000-2000µs) |
| Delay >200ms | Servo lag | Check servo quality, might need replacement |
| Jittery angle | Electrical noise | Check USB cable shielding, power supply |

---

## 🏎️ Throttle & Acceleration

### Expected Ranges (2S LiPo, small RC car)
| Measurement | Good | Acceptable | Bad |
|---|---|---|---|
| **Max velocity** | 1.0–2.5 m/s | 0.8–3.0 m/s | <0.5 m/s or >5 m/s |
| **Acceleration** | 0.5–2.0 m/s² | 0.3–3.0 m/s² | <0.2 m/s² or >5 m/s² |
| **Deceleration** | 0.2–1.0 m/s² | 0.1–2.0 m/s² | <0.05 m/s² (too slippery) |
| **Time to max** | 2–5 seconds | 1–8 seconds | <1s or >10s |

### Why These Ranges?
- **1-2 m/s:** Safe testing speed indoors; 2+ m/s too fast for tight mazes
- **0.5-2 m/s²:** Reasonable acceleration; matches wheel slip, motor power
- **Time to max:** Reflects motor response time and mechanical inertia

### If Measurements Are Off
| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Max velocity <0.5 m/s | Dead battery, motor damage | Charge battery, test motor directly |
| Acceleration <0.2 m/s² | Motor worn, gearing wrong | Check for wheel slip, inspect gears |
| Deceleration >1 m/s² | Brake engaged, high friction | Check brake, verify floor surface |
| Inconsistent runs | Wheel slip, dirty wheels | Clean wheels, test on same surface |

---

## 📡 LiDAR Sensor (Hokuyo 10LX)

### Expected Noise Profile
| Distance | Good Std Dev | Acceptable | Bad |
|----------|---|---|---|
| **1 meter** | 0.03–0.05m | 0.02–0.08m | >0.10m |
| **2 meters** | 0.03–0.06m | 0.02–0.10m | >0.15m |
| **5 meters** | 0.05–0.10m | 0.03–0.15m | >0.20m |
| **10 meters** | 0.08–0.15m | 0.05–0.25m | >0.30m |

### Linear Noise Model
- **Formula:** `noise_std = a + b * distance`
- **Good coefficient a:** 0.02–0.05 (baseline noise at 0m)
- **Good coefficient b:** 0.005–0.015 (noise growth per meter)

### Example of Good Data
```
Distance  Mean    Std     Error
1m        1.001m  0.040m  ±4cm
2m        2.002m  0.045m  ±4.5cm
5m        5.003m  0.075m  ±7.5cm
10m       10.005m 0.130m  ±13cm
```

### If Measurements Are Off
| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Noise >0.1m at 1m | Lens dirty, reflective surface | Clean lens, move away from shiny walls |
| Noise increases >0.02m per meter | Old sensor, electrical noise | Check cable, test elsewhere |
| Random dropouts (NaN values) | USB interference, power | Check USB cable, away from motors |
| Constant reading (no variance) | Stopped scan, misconfiguration | Check ROS topic, restart driver |

### Key Signs of Health
✓ Noise std increases gradually with distance (linear)  
✓ No sudden spikes or outliers (% outliers <1%)  
✓ Mean reading matches ground truth within ±2cm  
✓ Consistent across multiple measurements  

---

## 🧭 IMU (Accelerometer + Gyroscope)

### Expected Values

#### Gyroscope (at rest, STATIONARY car)
| Axis | Good | Acceptable | Bad |
|------|------|-----------|-----|
| **X, Y, Z bias** | <0.05 rad/s | <0.1 rad/s | >0.1 rad/s |
| **X, Y, Z noise** | <0.02 rad/s | <0.05 rad/s | >0.1 rad/s |

**In deg/s:**
- **Good bias:** <3 deg/s
- **Good noise:** <1 deg/s std dev

#### Accelerometer (at rest on flat ground)
| Axis | Good | Acceptable | Bad |
|------|------|-----------|-----|
| **Z axis** | 9.70–9.85 m/s² | 9.50–10.0 m/s² | <9.0 or >10.5 m/s² |
| **X, Y axes** | <0.2 m/s² | <0.5 m/s² | >1.0 m/s² |
| **Noise (all)** | <0.05 m/s² | <0.1 m/s² | >0.2 m/s² |

### Why These Ranges?
- **Gyro bias:** Should be ~0 (sensor measures rotation, standing still = no rotation)
- **Accel Z:** Gravity is 9.81 m/s² pointing down; ±0.15 is normal
- **Accel X,Y:** Should be 0 when stationary on flat surface

### If Measurements Are Off
| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Gyro bias >0.2 rad/s | Uncalibrated, ground not level | Recalibrate, test on flat surface |
| Accel Z way off (8 or 11) | Car tilted, sensor misaligned | Level car on flat ground, check mount |
| High noise (>0.2 m/s²) | Vibration from motor, loose mount | Check IMU mounting, reduce motor vibration |
| Drifting readings | Thermal drift (rare) | Warm up for 5 min, test again |

### Good Sign
✓ Accel X,Y < 0.3 m/s² (nearly zero)  
✓ Accel Z near 9.81 m/s² (gravity)  
✓ Gyro bias < 0.05 rad/s (very low)  
✓ Consistent across multiple measurements  

---

## ⏱️ Latency (End-to-End)

### Expected Ranges
| Measurement | Good | Acceptable | Bad |
|---|---|---|---|
| **LiDAR → SLAM** | 30–60ms | 20–100ms | >150ms |
| **Total (scan → motor)** | 50–80ms | 40–120ms | >200ms |
| **LiDAR frequency** | 24–26 Hz | 20–30 Hz | <15 Hz |

### Why These Ranges?
- **<100ms total:** Real-time control requirement for 1+ m/s speeds
- **SLAM latency <60ms:** Allows time for policy inference (20ms) + motor command (10ms)
- **25 Hz LiDAR:** Hokuyo 10LX spec; should be consistent

### If Measurements Are Off
| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| >150ms latency | Heavy SLAM computation, USB bottleneck | Reduce map resolution, check USB speed |
| Inconsistent latency (high variance) | Network jitter, CPU load | Check WiFi signal, close background apps |
| LiDAR frequency <20 Hz | Driver issue, USB bandwidth | Restart LiDAR driver, try USB 3.0 |
| Variable delays (20-200ms range) | Dropped packets, buffer issues | Hardwire Jetson (Ethernet), restart stack |

---

## ✅ Overall Sanity Check

**If ALL of these are true, Phase 1 is good:**

- [ ] Steering angle range ≥30° (both directions)
- [ ] Steering response delay <150ms
- [ ] Max velocity ≥0.8 m/s
- [ ] Max acceleration ≥0.3 m/s²
- [ ] LiDAR noise at 1m <0.08m
- [ ] Gyro bias <0.1 rad/s
- [ ] Accel Z bias within 9.5–10.0 m/s²
- [ ] Total latency <150ms

**If any are false:** Investigate that component before moving to Phase 2

---

## 📊 Data Collection Sheet

Use this during testing:

```
Date: _______________
Time: _______________
Jetson IP: _______________
Track temperature: _______________

STEERING
- Left angle: _____° (expect 30-40°)
- Right angle: _____° (expect 30-40°)
- Response delay: _____ms (expect 50-100ms)

THROTTLE
- Max velocity: _____ m/s (expect 1-2.5 m/s)
- Acceleration: _____ m/s² (expect 0.5-2 m/s²)
- Deceleration: _____ m/s² (expect 0.2-1 m/s²)

LIDAR
- Noise @ 1m: _____ m (expect 0.03-0.05)
- Noise @ 5m: _____ m (expect 0.05-0.10)
- Noise @ 10m: _____ m (expect 0.08-0.15)

IMU
- Gyro bias X: _____ rad/s (expect <0.05)
- Accel Z: _____ m/s² (expect 9.7-9.85)
- Noise level: good / acceptable / bad

LATENCY
- Total latency: _____ ms (expect <100ms)

OVERALL: ✓ PASS / ⚠️ NEEDS INVESTIGATION
```

---

**Save this guide and reference it during measurement collection!**
