# Pre-Flight Checklist for Phase 1 (Tomorrow)

**Print this and check off as you go**

---

## ✅ Hardware (At School, Before Testing)

### Vehicle
- [ ] Battery charged (measure voltage: should be ~7.4V for 2S LiPo)
- [ ] Motor responds to throttle command
- [ ] Steering servo responds and moves smoothly
- [ ] LiDAR powers on (LED indicator?)
- [ ] IMU responds (check: `ros2 topic echo /imu` gets data)
- [ ] Car drives in straight line (no hard-right bias)
- [ ] Clear track available (10m+ straight line for throttle test)

### Network
- [ ] WiFi/Ethernet working on Jetson
- [ ] Jetson IP known: __________ (run `hostname -I`)
- [ ] SSH access works: `ssh jetson "echo works"`
- [ ] ROS 2 running: `ros2 node list` shows nodes
- [ ] ROS_LOCALHOST_ONLY=0 set on both machines

### Safety
- [ ] Track clear of obstacles
- [ ] Person ready to manually control car if needed
- [ ] Battery cutoff accessible
- [ ] No people/animals in test area

---

## ✅ Software (At School, Before Testing)

### On Jetson
- [ ] Workspace built: `colcon build` completed
- [ ] Scripts present: `ls scripts/*.py` shows all 5 scripts
- [ ] Config template exists: `ls config/vehicle_params.yaml`
- [ ] ROS 2 sourced: `source install/setup.bash`

### On Desktop
- [ ] Can SSH to Jetson
- [ ] Can see Jetson topics: `ros2 topic list` works
- [ ] rviz2 installed (optional): `rviz2` launches

---

## 📋 Measurements to Collect (Print This Table)

**Record measurements here during tests:**

### Servo Test
| PWM | Angle (degrees) | Notes |
|-----|-----------------|-------|
| 1000 (LEFT FULL) | _____ | |
| 1200 | _____ | |
| 1350 | _____ | |
| 1500 (CENTER) | _____ | |
| 1650 | _____ | |
| 1800 | _____ | |
| 2000 (RIGHT FULL) | _____ | |

Response delay (from video): _____ ms

### Throttle Test
| Level | Max Velocity | Acceleration | Notes |
|-------|--------------|--------------|-------|
| 25% | _____ m/s | _____ m/s² | |
| 50% | _____ m/s | _____ m/s² | |
| 75% | _____ m/s | _____ m/s² | |
| 100% | _____ m/s | _____ m/s² | |
| Coast-down (0%) | _____ m/s² decel | | |

### LiDAR Noise Test
| Distance | Mean (m) | Std Dev (m) | Outliers % |
|----------|----------|-------------|-----------|
| 1m | _____ | _____ | _____ |
| 2m | _____ | _____ | _____ |
| 5m | _____ | _____ | _____ |
| 10m | _____ | _____ | _____ |

### Geometry
| Measurement | Value | Notes |
|-------------|-------|-------|
| Wheelbase | _____ m | front to rear axle |
| Track width | _____ m | left to right wheel |
| LiDAR height | _____ m | from ground |

### IMU (from script output)
| Parameter | Value | Expected |
|-----------|-------|----------|
| Gyro X bias | _____ | ~0 deg/s |
| Gyro Y bias | _____ | ~0 deg/s |
| Gyro Z bias | _____ | ~0 deg/s |
| Accel Z bias | _____ | ~9.81 m/s² |

---

## 🎯 Expected Values (Good Signs)

| Parameter | Good Range | Bad Sign |
|-----------|-----------|----------|
| Steering angle range | ±30° to ±45° | <±20° or >±50° |
| Max velocity | 1-2 m/s | <0.5 m/s or >3 m/s |
| Max acceleration | 0.5-2 m/s² | <0.2 m/s² or >5 m/s² |
| LiDAR noise @ 1m | 0.03-0.05m | >0.1m (too noisy) |
| Gyro bias | <0.1 rad/s | >0.1 rad/s (uncalibrated) |
| Servo response | 50-150ms | >200ms (too slow) |
| Total latency | 40-80ms | >150ms (too high) |

---

## ⏱️ Time Budget (Rough)

| Task | Time | Status |
|------|------|--------|
| Setup & checks | 10 min | |
| Geometry measurement | 5 min | |
| Servo test | 20 min | |
| Throttle test | 20 min | |
| LiDAR test | 15 min | |
| IMU calib | 5 min | |
| Latency test | 5 min | |
| **Total** | **80 min** | |

**Target: Finish by ________ (time)**

---

## 🔧 Troubleshooting Quick Reference

### "Script says: No odometry received"
→ Start SLAM first: `ros2 launch lio_sam run.launch.py`

### "Script says: No LiDAR data"
→ Check topic: `ros2 topic echo /scan` (should see ranges)

### "Car won't respond to commands"
→ Check PWM topic: `ros2 topic echo /servo/command`

### "SSH disconnects"
→ Run: `export ROS_LOCALHOST_ONLY=0` on both machines

### "Values look wrong"
→ Double-check phone level app is level (not tilted)

---

## 📸 Photos to Take

- [ ] Photo of wheelbase measurement
- [ ] Photo of steering angle at left/right extremes
- [ ] Photo of setup (for documentation)
- [ ] Video of servo step response (for latency measurement)

---

## 📝 Notes

```
Jetson IP: _______________
Track dimensions: _______________
Weather: _______________
Issues encountered: _______________
Next session priorities: _______________
```

---

## ✅ Sign-Off

Phase 1 measurement session completed: _______________
All values recorded in vehicle_params.yaml: YES / NO
Ready for Phase 2 (SLAM): YES / NO
