# 18-Week Project Timeline - All 6 Phases

**Complete roadmap for roboracer autonomous navigation research**

---

## 📅 Phase Overview

| Phase | Duration | Focus | Deliverables | Server Role |
|-------|----------|-------|--------------|-------------|
| **1** | Weeks 1-2 | Hardware System ID | vehicle_params.yaml | Data archive |
| **2** | Weeks 3-5 | LiDAR SLAM | Map dataset, calib results | Map storage |
| **3** | Weeks 4-6 | Simulation Setup | Isaac Lab config, sim2real | GPU training |
| **4** | Weeks 6-10 | Policy Learning | Trained models, logs | Main training hub |
| **5** | Weeks 11-14 | Deployment | Quantized models, inference | Model serving |
| **6** | Weeks 14-18 | Evaluation | Benchmarks, reports, paper | Analysis & archiving |

---

## 🚀 Phase 1: Hardware System Identification (Weeks 1-2)

### Goals
- Characterize vehicle dynamics (steering, throttle, kinematics)
- Calibrate sensors (LiDAR noise, IMU bias)
- Measure end-to-end latency
- Establish baseline measurements

### Weekly Breakdown

**Week 1: Initial Measurement**
```
Day 1 (Tomorrow):  System ID Testing at School
  ├─ 10:00  Setup & pre-flight checklist (10 min)
  ├─ 10:10  Geometry measurement (5 min)
  ├─ 10:15  Servo test (20 min)
  ├─ 10:35  Throttle test (20 min)
  ├─ 10:55  LiDAR noise test (15 min)
  ├─ 11:10  IMU calibration (5 min)
  ├─ 11:15  Latency measurement (5 min)
  └─ 11:30  DONE! vehicle_params.yaml filled

Day 2-3:  Validation & Refinement
  ├─ Repeat critical measurements (if needed)
  ├─ Fine-tune kinematics model
  └─ Verify all values in expected range
```

**Week 2: Integration & Baseline**
```
  ├─ SLAM on Jetson with calibrated params
  ├─ First mapping run
  ├─ Verify odometry accuracy
  └─ Archive all data to server
```

### Deliverables
- ✅ `vehicle_params.yaml` — Complete hardware characterization
- ✅ `servo_response_profile.csv` — PWM vs angle
- ✅ `throttle_dynamics.csv` — Speed/acceleration curves
- ✅ `lidar_noise_model.json` — Distance-dependent noise
- ✅ `imu_calibration.yaml` — Gyro/accel biases
- ✅ `latency_measurements.txt` — End-to-end timing

### Server Commands
```bash
ssh school-server
cd ~/roboracer_project/phase1_sysid

# Archive measurement data
mkdir -p measurements/$(date +%Y%m%d)
scp jetson:~/roboracer_ws/config/vehicle_params.yaml ./
scp jetson:~/roboracer_ws/measurements/* ./measurements/$(date +%Y%m%d)/

# Backup
tar -czf phase1_measurements_$(date +%Y%m%d).tar.gz measurements/
```

---

## 🗺️ Phase 2: LiDAR SLAM & Mapping (Weeks 3-5)

### Goals
- Collect LiDAR scans in controlled environment
- Generate 2D occupancy map using LIO-SAM
- Validate odometry accuracy (ground truth comparison)
- Create dataset for simulation

### Weekly Breakdown

**Week 3: Data Collection**
```
Day 1-2:  Collection runs
  ├─ Drive predetermined paths (marked on floor)
  ├─ Record ROS bags (LiDAR, IMU, odometry)
  ├─ Multiple runs for robustness
  └─ Ground truth: measure via GPS-denied methods

Day 3-5:  SLAM Processing
  ├─ Run LIO-SAM with calibrated params
  ├─ Generate occupancy grid map
  ├─ Evaluate loop closure
  └─ Inspect for drift/artifacts
```

**Week 4-5: Map Refinement**
```
  ├─ Validate map accuracy (compare to measured)
  ├─ Collect additional data if needed
  ├─ Convert map to Isaac Lab format
  ├─ Archive all bags and maps
  └─ Ready for simulation
```

### Deliverables
- ✅ `maze_map.pgm` — Occupancy map
- ✅ `maze_map.yaml` — Map config
- ✅ ROS bag files (10+ GB) — Raw sensor data
- ✅ `odometry_validation.csv` — Ground truth comparison
- ✅ `loop_closure_report.txt` — SLAM quality metrics

### Server Commands
```bash
ssh school-server
cd ~/roboracer_project/phase2_slam

# Sync large ROS bag files
rsync -avz jetson:~/roboracer_ws/logs/bagfiles/ ./bags/ --delete

# Store maps
cp jetson:~/roboracer_ws/maps/*.pgm ./maps/
cp jetson:~/roboracer_ws/maps/*.yaml ./maps/

# Archive
tar -czf slam_data_week3-5.tar.gz maps/ odometry_validation/
```

---

## 🔬 Phase 3: Simulation Environment Setup (Weeks 4-6)

### Goals
- Set up Isaac Lab simulator on school server
- Recreate physical maze in simulation
- Implement domain randomization
- Validate sim-to-real gap

### Weekly Breakdown

**Week 4: Isaac Lab Installation**
```
On server:
  ├─ Clone Isaac Lab (if not done)
  ├─ Install dependencies (GPU/CUDA)
  ├─ Verify IsaacLab environment loads
  └─ Test with simple example environment
```

**Week 5: Environment Creation**
```
  ├─ Create maze geometry from real map
  ├─ Implement robot model (Jetson Orin + sensors)
  ├─ Configure sensor simulators (LiDAR, IMU, camera)
  ├─ Implement domain randomization:
  │  ├─ Maze layout variation
  │  ├─ Friction variation
  │  ├─ Sensor noise
  │  ├─ Actuator delays
  │  └─ Dynamic obstacles
  └─ Validate observation space matches real car
```

**Week 6: Validation**
```
  ├─ Collect sim trajectories
  ├─ Compare distributions to real data
  ├─ Tune randomization parameters
  └─ Ready for policy learning
```

### Deliverables
- ✅ `isaac_task.py` — Environment implementation
- ✅ `domain_randomization.yaml` — Randomization config
- ✅ `robot_model.usd` — USD model of car
- ✅ Sim vs. Real comparison plots
- ✅ `readme_isaac_setup.md` — How to run

### Server Commands
```bash
ssh school-server
cd ~/roboracer_project/isaac_lab_env/IsaacLab

# Activate environment
source venv/bin/activate

# Create task
mkdir -p exts/roboracer_task/roboracer_task/tasks/
# ... copy task files ...

# Test it
python scripts/standalone/demo.py --task Isaac-Roboracer-Direct-v0

# Archive configs
cp -r exts/roboracer_task ~/roboracer_project/phase3_sim/isaac_configs/
```

---

## 🧠 Phase 4: Policy Learning (Weeks 6-10)

### Goals
- Train world model (Dreamer) on simulation data
- Fine-tune control policy (PPO) end-to-end
- Collect sim trajectories for training
- Log training metrics, save checkpoints

### Weekly Breakdown

**Week 6: Warm-Up Data Collection**
```
  ├─ Run random policy in simulation for 10k steps
  ├─ Collect trajectories (observations, actions, rewards)
  ├─ Store in replay buffer
  └─ Compute statistics for normalization
```

**Week 7-8: Dreamer Training**
```
Day 1-3:  Initial Training
  ├─ Start with pre-training (if available)
  ├─ Train world model: predict next obs from action
  ├─ Monitor reconstruction loss, KL divergence
  ├─ Save checkpoints every 1000 steps
  └─ Check convergence

Day 4-7:  Extended Training
  ├─ Continue until model loss plateaus
  ├─ Sample imagined trajectories from model
  ├─ Validate model predictions on held-out data
  └─ Save best checkpoint
```

**Week 9: Policy Optimization**
```
  ├─ Freeze world model
  ├─ Train PPO policy using model dynamics
  ├─ Curriculum learning: start in center, expand range
  ├─ Monitor episode reward, success rate
  └─ Save policy checkpoints
```

**Week 10: Fine-Tuning & Evaluation**
```
  ├─ Hyperparameter sweep (optional)
  ├─ Evaluate on test maze layouts
  ├─ Compute statistics: mean episode return, success rate
  └─ Archive final models
```

### Deliverables
- ✅ `dreamer_model.pth` — Trained world model
- ✅ `ppo_policy.pth` — Trained control policy
- ✅ `training_logs/` — TensorBoard logs
- ✅ `learning_curves.pdf` — Loss and reward plots
- ✅ Sim evaluation metrics (success rate, etc.)
- ✅ Training config and hyperparameters

### Server Commands
```bash
ssh school-server
cd ~/roboracer_project/phase4_learning

# Start training (background)
nohup python train_dreamer.py \
  --config configs/dreamer.yaml \
  --logdir ./logs/run_001 \
  > training.log 2>&1 &

# Monitor in real-time
tail -f training.log

# View tensorboard
tensorboard --logdir ./logs/

# Save best model
cp logs/run_001/checkpoint_best.pth ../shared/models/dreamer_v1.pth
cp logs/run_001/policy_best.pth ../shared/models/ppo_v1.pth
```

---

## 📦 Phase 5: Model Optimization & Deployment (Weeks 11-14)

### Goals
- Quantize models for edge deployment (Jetson)
- Prune unnecessary connections
- Create inference wrapper
- Validate on real hardware

### Weekly Breakdown

**Week 11: Quantization**
```
  ├─ Convert models to TensorFlow Lite / ONNX
  ├─ Quantize to INT8 (4× smaller, faster)
  ├─ Benchmark: latency vs accuracy trade-off
  ├─ Validate on Isaac Lab (should match)
  └─ Archive quantized models
```

**Week 12: Pruning & Optimization**
```
  ├─ Identify prunable layers (magnitude-based)
  ├─ Remove 30-50% of weights
  ├─ Fine-tune on simulation data (5 min)
  ├─ Benchmark again
  └─ Target: <100ms inference on Jetson
```

**Week 13: Integration**
```
  ├─ Create inference node for Jetson
  ├─ Integrate with ROS 2 pipeline
  ├─ Test subscription to /scan, publish /motor_cmd
  ├─ Profile memory & CPU usage
  └─ Package into Docker (optional)
```

**Week 14: Real-World Validation**
```
  ├─ Deploy to Jetson
  ├─ Test in simple maze
  ├─ Collect performance data
  ├─ Compare sim vs. real trajectories
  └─ Iterate if needed
```

### Deliverables
- ✅ `ppo_quantized.tflite` — Quantized policy (Jetson)
- ✅ `dreamer_quantized.onnx` — Quantized world model
- ✅ `inference_node.py` — ROS 2 inference wrapper
- ✅ Quantization benchmark report
- ✅ Real-world deployment guide
- ✅ Performance comparison (sim vs. real)

### Server Commands
```bash
ssh school-server
cd ~/roboracer_project/phase5_deploy

# Quantize models
python quantize.py \
  --model ../shared/models/ppo_v1.pth \
  --output ppo_quantized.tflite \
  --int8

# Test on Jetson
scp ppo_quantized.tflite jetson:~/roboracer_ws/models/
ssh jetson "python ~/roboracer_ws/scripts/inference_test.py"

# Store best version
cp ppo_quantized.tflite ../shared/models/ppo_final.tflite
```

---

## 🏁 Phase 6: Evaluation & Paper Writing (Weeks 14-18)

### Goals
- Run comprehensive benchmarks
- Analyze success rates, trajectory quality
- Compare to baselines
- Write final report/paper
- Archive all results

### Weekly Breakdown

**Week 14-15: Comprehensive Evaluation**
```
  ├─ Run 50+ episodes in real maze
  ├─ Collect success rate, trajectory length, execution time
  ├─ Measure power consumption (Jetson power profiler)
  ├─ Record video of successful/failed runs
  ├─ Compare to baseline (classic SLAM + RRT*)
  └─ Statistical analysis
```

**Week 16-17: Report Writing**
```
  ├─ Write main paper (6-8 pages)
  ├─ Generate results plots and tables
  ├─ Create demonstration videos
  ├─ Document lessons learned
  ├─ Discuss future improvements
  └─ Peer review (internal)
```

**Week 18: Final Submission**
```
  ├─ Submit to class/conference
  ├─ Create final presentations
  ├─ Archive everything
  └─ Celebrate! 🎉
```

### Deliverables
- ✅ Final report (PDF)
- ✅ Benchmark results table
- ✅ Plots: success rate, latency, power consumption
- ✅ Demo videos (successful runs)
- ✅ Code repository (GitHub)
- ✅ Reproducibility guide
- ✅ Data archive (all raw data)

### Server Commands
```bash
ssh school-server
cd ~/roboracer_project/phase6_eval

# Analyze results
python analyze_benchmarks.py \
  --logs ../shared/logs/ \
  --models ../shared/models/ \
  --output ./benchmark_report.html

# Generate plots
python generate_plots.py \
  --data ./evaluation_data.csv \
  --output ./figures/

# Create archive for submission
tar -czf roboracer_final_submission.tar.gz \
  ../../shared/ \
  ../*/  \
  --exclude="*.rosbag*" \
  --exclude="__pycache__"

# Upload to server for backup
cp roboracer_final_submission.tar.gz ~/roboracer_project/shared/archives/
```

---

## 📊 Master Task List

### Phase 1 (Weeks 1-2)
- [ ] Day 1: Run 5 system ID tests
- [ ] Day 2-3: Validate measurements
- [ ] Week 2: First SLAM run, verify odometry

### Phase 2 (Weeks 3-5)
- [ ] Week 3: Collect LiDAR data (10+ runs)
- [ ] Week 4: Run SLAM, generate maps
- [ ] Week 5: Validate accuracy, prepare for sim

### Phase 3 (Weeks 4-6)
- [ ] Week 4: Install Isaac Lab on server
- [ ] Week 5: Create environment, domain randomization
- [ ] Week 6: Validate sim-to-real gap

### Phase 4 (Weeks 6-10)
- [ ] Week 6: Collect warm-up trajectories
- [ ] Week 7-8: Train Dreamer world model
- [ ] Week 9: Optimize PPO policy
- [ ] Week 10: Archive final models

### Phase 5 (Weeks 11-14)
- [ ] Week 11: Quantize models
- [ ] Week 12: Prune and optimize
- [ ] Week 13: Integrate with ROS 2
- [ ] Week 14: Deploy and test on Jetson

### Phase 6 (Weeks 14-18)
- [ ] Week 14-15: Run benchmarks, collect data
- [ ] Week 16-17: Write report and analysis
- [ ] Week 18: Final submission

---

## 🔧 Critical Dependencies & Sequencing

```
Phase 1 (SysID)
    ↓
Phase 2 (SLAM) ←─────────────┐
    ↓                         │
Phase 3 (Sim Setup) ─────→ Phase 4 (Learn)
    ↓                         ↓
Phase 5 (Deploy) ←───────────┘
    ↓
Phase 6 (Eval & Paper)
```

**Cannot start Phase N until Phase N-1 delivers:**
- Phase 1 → Phase 2: vehicle_params.yaml
- Phase 2 → Phase 3: maze_map.pgm + ROS bags
- Phase 3 → Phase 4: Isaac Lab environment working
- Phase 4 → Phase 5: trained model checkpoints
- Phase 5 → Phase 6: quantized models on Jetson

---

## 🎯 Success Criteria

### Phase 1: "Green"
- [ ] All 6 measurements in expected range
- [ ] vehicle_params.yaml complete and validated

### Phase 2: "Green"
- [ ] SLAM map visually matches physical space
- [ ] Odometry error <5% over 50m trajectory

### Phase 3: "Green"
- [ ] Isaac Lab environment runs at 100Hz
- [ ] Sim observation distributions match real ±10%

### Phase 4: "Green"
- [ ] Sim success rate >80% on test mazes
- [ ] Dreamer reconstruction loss <0.1
- [ ] PPO reward converges

### Phase 5: "Green"
- [ ] Quantized model <20% accuracy loss
- [ ] Inference latency <100ms on Jetson

### Phase 6: "Green"
- [ ] Real-world success rate >60%
- [ ] Report complete and submitted

---

## 💾 Server Disk Space Estimate

```
Phase 1 data:      ~100 MB  (configs, calibration)
Phase 2 data:      ~50 GB   (ROS bags, maps)
Phase 3 data:      ~5 GB    (Isaac Lab configs)
Phase 4 data:      ~100 GB  (training logs, checkpoints)
Phase 5 data:      ~2 GB    (quantized models)
Phase 6 data:      ~20 GB   (evaluation videos, plots)
────────────────
Total:            ~177 GB

Recommendations:
• Minimum 300 GB SSD
• Archive old bags weekly
• Keep latest 2 training runs
• Compress videos (h264)
```

---

## 🚀 Staying on Schedule

**Weekly checkpoints (every Friday):**
```
Week 1:  Phase 1 complete ✓
Week 2:  Phase 1 validated ✓
Week 3:  Phase 2 started ✓
Week 4:  Phase 2 data complete, Phase 3 started ✓
Week 5:  Phase 2 complete, Phase 3 environment ✓
Week 6:  Phase 3 complete, Phase 4 training running ✓
Week 7:  Phase 4 in progress ✓
Week 8:  Phase 4 in progress ✓
Week 9:  Phase 4 policy training ✓
Week 10: Phase 4 complete, models saved ✓
Week 11: Phase 5 quantization done ✓
Week 12: Phase 5 optimization done ✓
Week 13: Phase 5 integration done ✓
Week 14: Phase 5 real-world validation, Phase 6 evals start ✓
Week 15: Phase 6 evaluations complete ✓
Week 16: Phase 6 paper drafted ✓
Week 17: Phase 6 paper complete ✓
Week 18: Final submission ✓
```

If you fall behind:
1. **Week 3 slip?** → Compress Phase 2 (use pre-made map)
2. **Week 6 slip?** → Use simulation-only policy (skip real-world Phase 2)
3. **Week 10 slip?** → Reduce quantization effort, use float32 on CPU
4. **Week 14 slip?** → Reduce evaluation breadth, focus on key metrics

---

**You've got this! The school server will make everything smooth. 🚀**
