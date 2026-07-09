# Tomorrow's Troubleshooting Guide

**If something breaks at school, start here**

---

## 🔴 Nothing Responds

### Checklist
1. **Is Jetson powered on?**
   ```bash
   ssh jetson "echo test"  # Should respond with "test"
   ```
   → If not, check power cable, battery

2. **Is ROS 2 running?**
   ```bash
   ros2 node list  # Should list nodes
   ```
   → If error, run: `source install/setup.bash`

3. **Are topics publishing?**
   ```bash
   ros2 topic list  # Should see: /scan, /imu, /odometry/filtered
   ```
   → If empty, start SLAM: `ros2 launch lio_sam run.launch.py`

4. **Is the car receiving commands?**
   ```bash
   ros2 topic echo /servo/command  # Should see PWM values
   ```
   → If not updating, script not running or topic wrong

---

## 🔴 Servo Test Fails

### Error: "No servo response"
```bash
# Check hardware
ros2 topic echo /servo/command  # Should see values (1000-2000 µs)
```
- **If values appear:** Motor connection issue, check PWM wire
- **If no values:** Script not publishing, try re-running script

### Error: "Can't measure angle"
- Bring phone level app (download before class)
- Make sure phone is level, not tilted
- Use protractor or angle measurement app

### Servo stuck at one angle
```bash
# Test directly
python3 scripts/servo_test.py
# Type: left → should move left
# Type: right → should move right
```
- **If moves:** Good, continue
- **If stuck:** Check servo mechanical limits, power supply

---

## 🔴 Throttle Test Fails

### Error: "No velocity data received"
```bash
# Check odometry
ros2 topic echo /odometry/filtered
```
- **If topic exists:** Velocity should update (linear.x field)
- **If topic missing:** SLAM not running

**Fix:**
```bash
ros2 launch lio_sam run.launch.py  # Start SLAM in separate terminal
# Wait 10 seconds, then retry throttle_test.py
```

### Error: "Car won't accelerate"
1. Check battery voltage (measure with multimeter if available)
   - 2S LiPo should be 7.4V (or 8.4V fresh)
   - If <6.8V, battery dead → charge

2. Test motor directly:
   ```bash
   # Publish manual command
   ros2 topic pub /motor/command std_msgs/msg/UInt16 "data: 1700"
   ```
   - **Car should move forward**
   - If not, check motor wiring

### Velocity readings look wrong (negative, or >10 m/s)
- **Check:** Is car moving? (should see velocity increasing)
- **Verify:** Odometry is from wheel encoders, not dead-reckoning
- **Measure:** Ground truth with tape measure (10m), compare

---

## 🔴 LiDAR Test Fails

### Error: "No LiDAR data"
```bash
ros2 topic echo /scan  # Should show ranges
```
- **If nothing:** LiDAR driver not started
  ```bash
  ros2 launch hokuyo_node urg_node.launch.py  # Start driver
  ```

- **If "Connection refused":** LiDAR USB not plugged in
  - Check USB cable to Jetson
  - Try different USB port

- **If data shows huge values (999m):** Lens dirty
  - Clean lens with soft cloth
  - Test from different distance

### Noise values way too high (>0.2m)
1. **Check surface:** Is wall reflective/shiny?
   - Move away from windows, mirrors
   - Test against matt wall

2. **Check interference:** Any electronics nearby?
   - Move away from WiFi router, motor
   - Test in quiet RF location

3. **Check hardware:** Is LiDAR mounted level?
   - Verify mounting isn't loose
   - Point at wall, not at angle

---

## 🔴 IMU Test Fails

### Error: "No IMU data" or "No readings"
```bash
ros2 topic echo /imu  # Should show angular_velocity and linear_acceleration
```
- **If nothing:** IMU not connected or driver not running
  - Check USB connection to Jetson
  - Verify ROS driver started

### Values look wrong (Z accel = 0, gyro = 1.0 rad/s)
1. **Check position:** Car must be on FLAT, LEVEL surface
   - Not tilted
   - Not on slope
   - Not on bumpy surface

2. **Check mounting:** IMU should be level with car
   - Not upside down
   - Not at angle

3. **Check motion:** Car must be STATIONARY
   - No vibration from motor
   - No one touching car
   - Wait 30 seconds for settling

### Calibration takes too long
- Script is normal; just wait
- It needs 60 seconds of data to be accurate
- After 1 minute, press Ctrl+C to finish

---

## 🔴 Script Crashes or Errors

### Error: "ModuleNotFoundError: No module named 'rclpy'"
```bash
# Forgot to source workspace
source install/setup.bash
# Retry script
python3 scripts/servo_test.py
```

### Error: "Cannot import numpy"
```bash
pip install numpy scipy
source install/setup.bash
# Retry
```

### Error: "Timeout waiting for first message"
- Some topic not publishing
- Check: `ros2 topic list | grep scan`
- Wait 5 seconds and retry
- If still fails, start SLAM separately:
  ```bash
  # Terminal 1
  ros2 launch lio_sam run.launch.py
  
  # Terminal 2 (after waiting 10s)
  python3 scripts/throttle_test.py
  ```

### Error: "Permission denied"
```bash
chmod +x scripts/*.py  # Make scripts executable
python3 scripts/servo_test.py  # Retry
```

---

## 🟡 Weird Measurements

### Measurements seem too good (all perfect values)
- Might be placeholder data or simulation
- Double-check physical setup
- Verify multiple times

### Measurements are inconsistent (different each run)
- **Servo:** Might be environmental
  - Run 3 times, average results
  - Check servo hasn't drifted

- **Throttle:** Depends on surface friction
  - Use same floor location
  - Repeat 2-3 times, average

- **LiDAR:** Depends on surface
  - Use same wall
  - Run 3 times from same distance

- **Latency:** Network jitter
  - Close other apps
  - Try wired Ethernet if available
  - Run multiple times, average

---

## 🟡 Time Running Out

### What to skip if pressed for time

**Keep (critical):**
- Geometry measurement (5 min)
- Servo test (20 min)
- Throttle test (20 min)

**Can skip:**
- IMU calibration (optional, good default is near-zero bias)
- Latency test (use default 50ms if needed)
- Fine-tuning LiDAR at every distance (just do 1m and 5m)

---

## ✅ "It works!" - What's Next?

Once a test passes:
1. **Record value** in vehicle_params.yaml
2. **Move to next test** (don't re-run unless needed)
3. **Verify:** Check value is in "good" range (see EXPECTED_MEASUREMENTS.md)

---

## 🆘 Last Resort: Recovery Steps

### If everything is broken
1. **Restart Jetson:**
   ```bash
   ssh jetson "sudo reboot"
   # Wait 30 seconds
   # Try again
   ```

2. **Restart ROS 2:**
   ```bash
   # Kill all ROS processes
   killall -9 ros2
   pkill -f python
   
   # Rebuild workspace
   cd ~/roboracer_ws
   colcon build --symlink-install
   source install/setup.bash
   
   # Try script again
   ```

3. **Test minimal setup:**
   ```bash
   # Just check if sensors are alive
   ros2 topic echo /scan        # LiDAR?
   ros2 topic echo /imu         # IMU?
   ros2 topic echo /odometry/filtered  # SLAM?
   ```

4. **If sensors not publishing:**
   - Check USB cables
   - Restart drivers
   - Try different USB ports

---

## 📞 Emergency Contact

If stuck for >10 minutes on one test:
1. Skip that test (some measurements are optional)
2. Mark in vehicle_params.yaml: "skipped, used default"
3. Move to next test
4. Come back to it if time permits

---

## 📝 Success Markers

You'll know it's working when:
- ✓ `ros2 node list` shows multiple nodes
- ✓ `ros2 topic list` shows /scan, /imu, /odometry/filtered
- ✓ Scripts run without crashing
- ✓ Measurements are in expected range
- ✓ Can complete Phase 1 in ~75 minutes

---

## 🎯 If You Get Stuck

**Check in this order:**
1. Is Jetson on? (`ssh jetson "echo test"`)
2. Is ROS 2 sourced? (`ros2 node list`)
3. Are topics publishing? (`ros2 topic list`)
4. Is the specific topic data? (`ros2 topic echo /topic`)
5. Is the script running? (Check terminal output)

**Most common fixes:**
- `source install/setup.bash`
- `ros2 launch lio_sam run.launch.py` (start SLAM)
- Check USB cable connections
- Power cycle Jetson

---

**Save this file and bring it tomorrow!**
