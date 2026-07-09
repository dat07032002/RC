# Roboracer Starter Code Package

**Everything you need to start Phase 1 System Identification TODAY**

All files are in this directory and ready to copy to your Jetson/desktop.

---

## 📦 Files Included

### Configuration
- **`vehicle_params.yaml`** — Template with all parameters to fill in during Phase 1
  - Copy to: `~/roboracer_ws/config/vehicle_params.yaml`

### Python Scripts (Phase 1 System ID)
- **`servo_test.py`** — Interactive steering servo testing
  - Measures: gain, response delay, max angles
  - Copy to: `~/roboracer_ws/scripts/`

- **`throttle_test.py`** — Throttle response and acceleration testing
  - Measures: max velocity, acceleration, coast-down friction
  - Copy to: `~/roboracer_ws/scripts/`

- **`lidar_noise_test.py`** — LiDAR noise characterization
  - Measures: noise at 1m, 2m, 5m, 10m distances
  - Copy to: `~/roboracer_ws/scripts/`

- **`imu_calib.py`** — IMU bias and noise calibration
  - Measures: gyro bias, accel bias, noise levels
  - Copy to: `~/roboracer_ws/scripts/`

- **`latency_test.py`** — End-to-end latency profiling
  - Measures: LiDAR → SLAM → Policy delay
  - Copy to: `~/roboracer_ws/scripts/`

### ROS 2
- **`system_id.launch.py`** — Launch file for Phase 1 testing
  - Copy to: `~/roboracer_ws/launch/`

### Setup
- **`setup.sh`** — One-time workspace initialization script
  - Run once: `bash setup.sh`

---

## 🚀 Quick Start (5 minutes)

### On Your Desktop/Laptop

1. **Copy all files:**
   ```bash
   cd ~/roboracer_ws
   
   # Copy scripts
   cp /path/to/starter_code/*.py scripts/
   chmod +x scripts/*.py
   
   # Copy config
   cp /path/to/starter_code/vehicle_params.yaml config/
   
   # Copy launch file
   cp /path/to/starter_code/system_id.launch.py launch/
   
   # Run setup
   bash /path/to/starter_code/setup.sh
   ```

2. **Update SSH config:**
   ```bash
   # Find Jetson IP (at school)
   ping jetson.local  # or check router
   
   # Add to ~/.ssh/config
   cat >> ~/.ssh/config << 'EOF'
   
   Host jetson
       HostName 192.168.x.x  # REPLACE!
       User jetson
       Port 22
   EOF
   
   # Test
   ssh jetson "ros2 --version"
   ```

### On Jetson (at school)

1. **Verify ROS 2 installed:**
   ```bash
   ros2 --version
   ```

2. **Enable distributed ROS 2** (if needed):
   ```bash
   echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
   echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
   source ~/.bashrc
   ```

---

## 📝 Phase 1 Workflow (Tomorrow at School)

### Step 1: Geometry Measurements (5 min)
```bash
# On desktop via SSH
ssh jetson

# Manually measure wheelbase, track width
# Record in: ~/roboracer_ws/config/vehicle_params.yaml
```

### Step 2: Servo Testing (20 min)
```bash
ssh jetson
cd ~/roboracer_ws
source install/setup.bash
python3 scripts/servo_test.py
```
- Follow prompts
- Measure angles with phone level app
- Record results in `vehicle_params.yaml`

### Step 3: Throttle Testing (20 min)
```bash
python3 scripts/throttle_test.py
```
- Tests at 25%, 50%, 75%, 100% throttle
- Measures max velocity and acceleration
- Record results in `vehicle_params.yaml`

### Step 4: LiDAR Noise Testing (15 min)
```bash
python3 scripts/lidar_noise_test.py
```
- Position car at 1m, 2m, 5m, 10m from wall
- Collects noise statistics at each distance
- Record results in `vehicle_params.yaml`

### Step 5: IMU Calibration (5 min)
```bash
python3 scripts/imu_calib.py
```
- Car must be STATIONARY
- Measures gyro bias, accel bias, noise
- Record results in `vehicle_params.yaml`

### Step 6: Latency Profiling (5 min)
```bash
python3 scripts/latency_test.py
```
- Drive car slowly around maze
- Measures end-to-end latency
- Record results in `vehicle_params.yaml`

### Step 7: System ID Report (30 min analysis at home)
- Fill in all XXX values in `vehicle_params.yaml`
- Generate final report
- **Phase 1 COMPLETE** ✓

---

## ✅ Expected Time

| Task | Time |
|------|------|
| Setup (one-time) | 5 min |
| Geometry measurement | 5 min |
| Servo test | 20 min |
| Throttle test | 20 min |
| LiDAR noise test | 15 min |
| IMU calibration | 5 min |
| Latency profiling | 5 min |
| **Total at school** | **75 min** |
| Analysis at home | 30 min |
| **TOTAL PHASE 1** | **~2 hours** |

---

## 📋 Troubleshooting

### "No velocity data received"
- Check: `ros2 topic list | grep odom`
- Verify: `/odometry/filtered` topic exists
- Start SLAM: `ros2 launch lio_sam run.launch.py`

### "No LiDAR data"
- Check: `ros2 topic list | grep scan`
- Verify: `/scan` topic is publishing
- Clean lens and test again

### "SSH connection refused"
- Find Jetson IP: `ping jetson.local` or router admin
- Update `~/.ssh/config` with correct IP
- Test: `ssh jetson "ls ~"`

### "Python import errors"
- Install: `pip install numpy scipy`
- Build workspace: `colcon build`
- Source: `source install/setup.bash`

---

## 📖 Reference

- **Full implementation guide:** `~/roboracer_ws/implementation_guide.md` Phase 1
- **Technical details:** `~/roboracer_ws/comprehensive_plan.md`
- **System ID checklist:** Check the system ID checklist artifact

---

## 🎯 Next Steps After Phase 1

Once `vehicle_params.yaml` is complete:

1. **Phase 2:** Deploy LiDAR SLAM
2. **Phase 3:** Set up Isaac Lab simulation
3. **Phase 4:** Train Dreamer + PPO policies

All documented in `implementation_guide.md`

---

## 💬 Questions?

Check `implementation_guide.md` for each specific section:
- Phase 1 → Detailed procedures
- Troubleshooting section → Common issues
- Remote access setup → SSH and ROS 2 distributed
