# Roboracer: Autonomous Navigation for GPS-Denied Environments

A research project combining LiDAR-based SLAM, reinforcement learning (Dreamer + PPO), and sim-to-real transfer for autonomous maze navigation on edge hardware (Jetson Orin Nano).

## Quick Start

### Phase 1: Hardware System ID (Week 1-2)
```bash
# On Jetson Orin Nano
cd phase1_sysid
python scripts/servo_test.py
python scripts/throttle_test.py
python scripts/lidar_noise_test.py
python scripts/imu_calib.py
python scripts/latency_test.py
```

### Phase 4: Train Policy (Week 6-10)
```bash
# On UT training server
source activate.sh
cd phase4_learning
python train_policy.py --steps 100000 --gpu 0
```

## Project Structure

```
├── phase1_sysid/      Hardware characterization
├── phase2_slam/       LiDAR SLAM mapping
├── phase3_sim/        Simulation environment
├── phase4_learning/   Policy training (PPO)
├── phase5_deploy/     Model quantization & deployment
├── phase6_eval/       Evaluation & benchmarking
└── docs/              Complete documentation
```

## Hardware

- **Robot**: Jetson Orin Nano + Hokuyo 10LX LiDAR + Intel RealSense Camera + custom servo/throttle
- **Training Server**: 5x RTX 6000 Ada GPUs, 500GB storage
- **Framework**: ROS 2, Isaac Lab, Stable-Baselines3

## Documentation

- [Implementation Guide](docs/PHASES_TIMELINE.md) - 18-week detailed plan
- [Server Setup](docs/SERVER_SETUP_GUIDE.md) - Infrastructure details
- [Quick Reference](docs/QUICK_REFERENCE.md) - Phase 1 cheat sheet
- [School SLAM Test Plan](docs/SCHOOL_SLAM_TEST_PLAN.md) - mapping, localization, and obstacle-preview checklist
- [Expected Measurements](docs/EXPECTED_MEASUREMENTS.md) - Hardware validation

## 18-Week Timeline

| Phase | Duration | Focus |
|-------|----------|-------|
| 1 | Weeks 1-2 | Hardware system identification |
| 2 | Weeks 3-5 | LiDAR SLAM mapping |
| 3 | Weeks 4-6 | Simulation environment setup |
| 4 | Weeks 6-10 | Policy learning (Dreamer + PPO) |
| 5 | Weeks 11-14 | Model optimization & deployment |
| 6 | Weeks 14-18 | Evaluation & paper writing |

## Key Technologies

- **RL**: Dreamer world model + PPO control policy
- **SLAM**: LIO-SAM (LiDAR-Inertial Odometry)
- **Simulation**: Isaac Lab (NVIDIA Omniverse)
- **Edge Deployment**: TensorFlow Lite quantization
- **Framework**: ROS 2 Humble, Stable-Baselines3

## Files

### Phase 1 (Hardware Testing)
- `servo_test.py` - Steering servo characterization
- `throttle_test.py` - Velocity & acceleration profiling
- `lidar_noise_test.py` - Sensor noise characterization
- `imu_calib.py` - IMU bias calibration
- `latency_test.py` - End-to-end system latency

### Configuration
- `vehicle_params.yaml` - Hardware parameters template

### Documentation
- `PHASES_TIMELINE.md` - Complete 18-week breakdown
- `SERVER_SETUP_GUIDE.md` - Server infrastructure
- `QUICK_REFERENCE.md` - Phase 1 quick start
- `SCHOOL_SLAM_TEST_PLAN.md` - School mapping/localization checklist
- `PREFLIGHT_CHECKLIST.md` - Hardware verification
- `EXPECTED_MEASUREMENTS.md` - Sanity check ranges

## Status

✅ Phase 1: Framework ready  
✅ Phase 4: Training infrastructure set up on UT server  
⏳ Week 1-2: Hardware testing  
⏳ Week 3-5: SLAM mapping  
⏳ Week 4-6: Simulation  
⏳ Week 6-10: Policy training  

## Author

Thanh Dat  
thanhdatwave@gmail.com

## License

MIT

