# Quick Reference Card - Phase 1 (Print This!)

**One-page cheat sheet for tomorrow**

---

## 🚀 Quick Start

```bash
# On Jetson (SSH from desktop)
ssh jetson
cd ~/roboracer_ws
source install/setup.bash

# Run tests in order:
python3 scripts/servo_test.py       # Step 1: Steering
python3 scripts/throttle_test.py    # Step 2: Acceleration  
python3 scripts/lidar_noise_test.py # Step 3: LiDAR
python3 scripts/imu_calib.py        # Step 4: IMU
python3 scripts/latency_test.py     # Step 5: Timing

# Fill in measurements → vehicle_params.yaml
# Done!
```

---

## 📱 Commands Quick Lookup

| What | Command |
|------|---------|
| Find Jetson IP | `ping jetson.local` |
| SSH to Jetson | `ssh jetson` |
| Check topics | `ros2 topic list` |
| Monitor topic | `ros2 topic echo /topic` |
| See current latency | `ros2 topic hz /scan` |
| Check ROS 2 | `ros2 node list` |
| View errors | `ros2 doctor` |

---

## 📊 What Each Script Does

### servo_test.py
- **Goal:** Measure steering angle at different PWM values
- **Time:** ~20 min
- **What you do:**
  1. Run script
  2. Type "left" → measure angle with level app
  3. Type "center" → measure angle
  4. Type "right" → measure angle
  5. Type "exit"
- **Record:** angle_left_max, angle_right_max

### throttle_test.py
- **Goal:** Measure velocity and acceleration
- **Time:** ~20 min
- **What you do:**
  1. Run script
  2. Car accelerates at 25%, 50%, 75%, 100%
  3. Script prints max velocity and acceleration
- **Record:** max_velocity, max_acceleration, friction_coefficient

### lidar_noise_test.py
- **Goal:** Measure LiDAR noise at different distances
- **Time:** ~15 min
- **What you do:**
  1. Position car 1m from wall
  2. Run script, wait for data
  3. Move to 2m, 5m, 10m, repeat
- **Record:** noise_std_at_1m, noise_std_at_5m, etc.

### imu_calib.py
- **Goal:** Measure IMU calibration offsets
- **Time:** ~5 min
- **What you do:**
  1. Place car STATIONARY on flat ground
  2. Run script, wait 60 seconds
  3. Do NOT move car
- **Record:** gyro_bias, accel_bias

### latency_test.py
- **Goal:** Measure end-to-end latency
- **Time:** ~5 min
- **What you do:**
  1. Run script
  2. Drive car slowly around maze for 2-3 min
  3. Press Ctrl+C
  4. Script analyzes delays
- **Record:** lidar_to_slam_ms, total_latency_ms

---

## 📋 Measurement Template (Copy to vehicle_params.yaml)

```yaml
steering:
  angle_left_max: XXX
  angle_right_max: XXX
  servo_response_delay: XXX

throttle:
  max_velocity: X.XX
  max_acceleration: X.XX
  friction_coefficient: -X.XX

sensors:
  lidar:
    noise_std_at_1m: 0.XXX
    noise_std_coefficient_a: 0.XXX
    noise_std_coefficient_b: 0.XXX
  
  imu:
    gyro_bias: [X.XXX, X.XXX, X.XXX]
    accel_bias: [X.XXX, X.XXX, X.XXX]

latency:
  lidar_to_slam_ms: XXX
  total_latency_ms: XXX
```

---

## ⚠️ Common Issues & Fixes

| Issue | Fix |
|-------|-----|
| "No odometry" | Start SLAM: `ros2 launch lio_sam run.launch.py` |
| "No LiDAR scan" | Check: `ros2 topic echo /scan` |
| "SSH won't connect" | Update ~/.ssh/config with correct IP |
| "Steering won't move" | Check: `ros2 topic echo /servo/command` |
| "Values way off" | Verify phone level app is working |
| "Script crashes" | Run: `source install/setup.bash` first |

---

## 🎯 Success Criteria (Sanity Check)

✓ Steering angle range: 30°–45°  
✓ Max velocity: 1–2 m/s  
✓ Max acceleration: 0.5–2 m/s²  
✓ LiDAR noise: 0.03–0.05m at 1m  
✓ Gyro bias: <0.1 rad/s  
✓ Total latency: <100ms  

**If all green:** Phase 1 DONE ✓

---

## 📞 Stuck? Check This

1. Is Jetson on? (blinking LED?)
2. Is ROS 2 running? (`ros2 node list`)
3. Is LiDAR publishing? (`ros2 topic echo /scan`)
4. Is IMU publishing? (`ros2 topic echo /imu`)
5. Can you SSH? (`ssh jetson "echo test"`)

---

## ⏱️ Timeline

```
10:00 → Arrive, setup (10 min)
10:10 → Servo test (20 min)
10:30 → Throttle test (20 min)
10:50 → LiDAR test (15 min)
11:05 → IMU calib (5 min)
11:10 → Latency test (5 min)
11:15 → Pack up, DONE!
```

**Total: 75 minutes**

---

## 📸 Photos to Take

- [ ] Wheelbase measurement
- [ ] Servo at full left/right
- [ ] Setup overview
- [ ] Servo response video (slow-mo, 120fps)

---

## 🔗 Reference

- Full guide: `/home/jetson/roboracer_ws/implementation_guide.md`
- Config template: `/home/jetson/roboracer_ws/config/vehicle_params.yaml`
- Scripts: `/home/jetson/roboracer_ws/scripts/`

---

**Print this card, bring it tomorrow!**
