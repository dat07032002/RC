# School Server Setup Guide - Complete Project Infrastructure

**Central hub for all 6 phases of the roboracer project**

---

## 🏗️ Architecture Overview

```
JETSON ORIN (At School)          SCHOOL SERVER (Centralized)      YOUR DESKTOP
──────────────────────           ──────────────────────────       ──────────────
ROS 2 Core                        ROS 2 Network Hub                Optional Monitor
├─ SLAM node                      ├─ Isaac Lab (GPU training)      ├─ rviz2 visualization
├─ Policy inference               ├─ Training scripts              ├─ Data analysis
├─ Motor control                  ├─ Data storage                  └─ File transfers
└─ Sensors                        ├─ Model repository
                                  ├─ Bag file archive
                                  └─ Log aggregation

                              ↕↕↕ ROS 2 Distributed Network
                          (Same school network, low latency)
```

---

## 📋 Phase-by-Phase Server Usage

| Phase | What Runs on Server | Purpose |
|-------|---------------------|---------|
| **1-2** | Storage + SSH access | Archive hardware data, remote monitoring |
| **3-4** | Isaac Lab + training | Train policies, store models, run hyperparameter searches |
| **5** | Model serving | Deploy trained models back to Jetson |
| **6** | Data analysis | Benchmark reports, visualization, paper generation |

---

## 🔑 Part 1: SSH Key Setup (Security Best Practice)

### Generate SSH Key (One-time)

**On your desktop:**
```bash
# Generate key pair (replace with your email)
ssh-keygen -t ed25519 -C "your_email@school.edu" -f ~/.ssh/server_key

# This creates:
# ~/.ssh/server_key (PRIVATE - never share)
# ~/.ssh/server_key.pub (PUBLIC - paste on server)
```

### Add Public Key to Server

**Step 1: Connect to server (password auth for first time)**
```bash
ssh username@server.ip.address
# Enter password when prompted
```

**Step 2: Create SSH directory and add key**
```bash
# On server
mkdir -p ~/.ssh
chmod 700 ~/.ssh

# Paste your public key content:
echo "ssh-ed25519 AAAAC3NzaC1..." >> ~/.ssh/authorized_keys
chmod 600 ~/.ssh/authorized_keys
```

**Step 3: Test passwordless login**
```bash
# On your desktop
ssh -i ~/.ssh/server_key username@server.ip.address
# Should connect WITHOUT password
```

**Step 4: Create SSH config for easy access**
```bash
# Edit ~/.ssh/config
cat >> ~/.ssh/config << 'EOF'

Host school-server
    HostName server.ip.address
    User username
    IdentityFile ~/.ssh/server_key
    Port 22
EOF

# Now use: ssh school-server
```

---

## 💾 Part 2: Server Directory Structure

**Create unified project workspace:**

```bash
ssh school-server

# Create project root
mkdir -p ~/roboracer_project
cd ~/roboracer_project

# Create subdirectories for each phase
mkdir -p {phase1_sysid, phase2_slam, phase3_sim, phase4_learning, phase5_deploy, phase6_eval}
mkdir -p shared/{data, models, datasets, logs, scripts, configs}
mkdir -p isaac_lab_env

# Set permissions
chmod 755 ~/roboracer_project
```

**Directory structure:**
```
~/roboracer_project/
├── phase1_sysid/        # Phase 1 hardware measurements
│   └── vehicle_params.yaml
├── phase2_slam/         # Phase 2 SLAM data
│   └── maps/
├── phase3_sim/          # Phase 3 simulation configs
│   └── isaac_lab_configs/
├── phase4_learning/     # Phase 4 training outputs
│   ├── models/
│   ├── training_logs/
│   └── datasets/
├── phase5_deploy/       # Phase 5 deployment artifacts
│   └── quantized_models/
├── phase6_eval/         # Phase 6 evaluation results
│   ├── benchmarks/
│   └── reports/
└── shared/
    ├── data/            # All collected data
    ├── models/          # All trained models
    ├── datasets/        # Isaac Lab datasets
    ├── logs/            # ROS bags, training logs
    ├── scripts/         # Shared Python scripts
    └── configs/         # Shared config files
```

---

## 🔗 Part 3: ROS 2 Network Configuration

### Enable ROS 2 Distributed Communication

**On Server:**
```bash
ssh school-server

# Add to ~/.bashrc
cat >> ~/.bashrc << 'EOF'

# ROS 2 Distributed Setup
export ROS_DOMAIN_ID=0                    # Same ID on all machines
export ROS_LOCALHOST_ONLY=0               # Allow network communication
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

EOF

source ~/.bashrc
```

**On Jetson (at school):**
```bash
ssh jetson

# Add same settings to ~/.bashrc
cat >> ~/.bashrc << 'EOF'

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

EOF

source ~/.bashrc
```

**On Your Desktop:**
```bash
# Add to ~/.bashrc or ~/.zshrc
cat >> ~/.bashrc << 'EOF'

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

EOF

source ~/.bashrc
```

### Verify Network Communication

```bash
# From server, check if can see Jetson nodes
ros2 node list
# Should show nodes from Jetson like: /lio_sam, /policy_node

# From desktop, check if can see server and Jetson
ros2 topic list
# Should show all topics from both server and Jetson
```

---

## 📦 Part 4: Install Dependencies on Server

### Prerequisites

```bash
ssh school-server

# Update system
sudo apt update && sudo apt upgrade -y

# Install ROS 2 (if not already installed)
# Follow: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# Install Python development tools
sudo apt install -y python3-pip python3-dev python3-venv

# Install build tools
sudo apt install -y build-essential cmake git

# Install machine learning libraries
pip install --upgrade pip
pip install numpy scipy scikit-learn matplotlib pandas
pip install torch torchvision  # Or tensorflow, depending on preference
```

### Isaac Lab Installation (If GPU Available)

```bash
ssh school-server
cd ~/roboracer_project/isaac_lab_env

# Clone Isaac Lab
git clone https://github.com/isaac-sim/IsaacLab.git
cd IsaacLab

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install Isaac Lab
pip install -e .

# Verify installation
python3 -c "from isaaclab.envs import gymnasium as gym; print('Isaac Lab installed!')"
```

---

## 📂 Part 5: Data Management & Synchronization

### Automated Data Sync from Jetson to Server

**Create sync script on Server:**

```bash
# File: ~/roboracer_project/shared/scripts/sync_from_jetson.sh

#!/bin/bash

JETSON_USER="jetson"
JETSON_IP="192.168.x.x"  # Update with actual IP
JETSON_PATH="~/roboracer_ws"
SERVER_PATH="~/roboracer_project/shared/data"

echo "Syncing from Jetson to Server..."

# Sync system ID data
rsync -avz "$JETSON_USER@$JETSON_IP:$JETSON_PATH/config/" "$SERVER_PATH/phase1_sysid/" --delete

# Sync SLAM maps
rsync -avz "$JETSON_USER@$JETSON_IP:$JETSON_PATH/maps/" "$SERVER_PATH/phase2_slam/" --delete

# Sync ROS bags
rsync -avz "$JETSON_USER@$JETSON_IP:$JETSON_PATH/logs/" "$SERVER_PATH/logs/" --delete

# Sync odometry data
rsync -avz "$JETSON_USER@$JETSON_IP:$JETSON_PATH/datasets/" "$SERVER_PATH/datasets/" --delete

echo "Sync complete!"
```

**Make executable and schedule:**

```bash
chmod +x ~/roboracer_project/shared/scripts/sync_from_jetson.sh

# Run manually:
~/roboracer_project/shared/scripts/sync_from_jetson.sh

# Or schedule with cron (sync every hour):
crontab -e
# Add line:
0 * * * * ~/roboracer_project/shared/scripts/sync_from_jetson.sh >> ~/roboracer_project/shared/logs/sync.log
```

---

## 🚀 Part 6: Phase-by-Phase Server Workflow

### Phase 1-2: Hardware Measurement (Weeks 1-5)

```bash
ssh school-server

# Create phase 1 directory
cd ~/roboracer_project/phase1_sysid

# Download system ID scripts from Jetson
scp jetson:~/roboracer_ws/scripts/*.py ./
scp jetson:~/roboracer_ws/config/vehicle_params.yaml ./

# Archive results after testing
tar -czf phase1_results_$(date +%Y%m%d).tar.gz vehicle_params.yaml measurements/
```

### Phase 3: Isaac Lab Setup (Week 4-6)

```bash
ssh school-server
cd ~/roboracer_project/isaac_lab_env/IsaacLab

# Activate environment
source venv/bin/activate

# Create symlink to shared data
ln -s ~/roboracer_project/shared/data ./data

# Create training directory
mkdir -p ~/roboracer_project/phase4_learning/training_jobs
```

### Phase 4: Training Job Submission (Week 6-10)

```bash
ssh school-server
cd ~/roboracer_project/phase4_learning

# Create training script
cat > train_dreamer.py << 'EOF'
# Training script here
# Will read from ~/roboracer_project/shared/data
# Will save models to ~/roboracer_project/shared/models
EOF

# Run training (in background, can close SSH)
nohup python3 train_dreamer.py > training.log 2>&1 &

# Monitor progress
tail -f training.log
```

### Phase 5: Model Deployment (Week 11-14)

```bash
ssh school-server
cd ~/roboracer_project/phase5_deploy

# Download trained models
cp ~/roboracer_project/shared/models/ppo_quantized.tflite ./

# Transfer to Jetson
scp ppo_quantized.tflite jetson:~/roboracer_ws/models/
```

### Phase 6: Analysis & Reporting (Week 14-18)

```bash
ssh school-server
cd ~/roboracer_project/phase6_eval

# Analyze results
python3 analyze_benchmarks.py \
  --models ~/roboracer_project/shared/models/ \
  --logs ~/roboracer_project/shared/logs/ \
  --output ./benchmark_report.html
```

---

## 🔄 Part 7: Backup & Archiving Strategy

### Automated Backup

```bash
# File: ~/roboracer_project/shared/scripts/backup.sh
#!/bin/bash

BACKUP_DIR="/mnt/backup/roboracer_backups"
PROJECT_DIR="~/roboracer_project"
DATE=$(date +%Y%m%d_%H%M%S)

# Backup critical files
tar -czf "$BACKUP_DIR/roboracer_$DATE.tar.gz" \
  "$PROJECT_DIR/shared/" \
  "$PROJECT_DIR/*/vehicle_params.yaml" \
  --exclude="*.tar.gz" \
  --exclude="logs/*" \
  --exclude="*.rosbag*"

# Keep only last 10 backups
ls -t "$BACKUP_DIR"/roboracer_*.tar.gz | tail -n +11 | xargs rm -f

echo "Backup complete: $BACKUP_DIR/roboracer_$DATE.tar.gz"
```

### Weekly Backup Schedule

```bash
# Add to crontab
crontab -e

# Run backup every Sunday at 2 AM
0 2 * * 0 ~/roboracer_project/shared/scripts/backup.sh >> ~/roboracer_project/shared/logs/backup.log
```

---

## 📊 Part 8: Monitoring & Logging

### Centralized Log Aggregation

```bash
# All processes should log to:
# ~/roboracer_project/shared/logs/

# Create log directories
mkdir -p ~/roboracer_project/shared/logs/{hardware,slam,training,deployment}

# In your scripts, use:
LOG_FILE="~/roboracer_project/shared/logs/training/phase4_$(date +%Y%m%d_%H%M%S).log"
python3 train.py 2>&1 | tee "$LOG_FILE"
```

### Monitor Long-Running Jobs

```bash
# Check if training is running
ps aux | grep python3 | grep train

# Monitor GPU usage (if available)
watch -n 1 nvidia-smi

# View recent logs
tail -f ~/roboracer_project/shared/logs/training/*.log
```

---

## 🔐 Part 9: Access Control & Permissions

### Set Proper Permissions

```bash
ssh school-server
cd ~/roboracer_project

# Owner can read/write, group can read, others nothing
chmod -R 750 .
chmod -R 640 ./shared/configs/  # Sensitive configs

# Make scripts executable
chmod +x ./shared/scripts/*.sh
```

### Team Access (If Multiple People)

```bash
# Create group
sudo groupadd roboracer_team

# Add users
sudo usermod -a -G roboracer_team username1
sudo usermod -a -G roboracer_team username2

# Set group permissions
chgrp -R roboracer_team ~/roboracer_project
chmod -R g+rwX ~/roboracer_project
```

---

## 📝 Quick Reference

### Common Commands

```bash
# SSH into server
ssh school-server

# SSH into Jetson from anywhere
ssh jetson  # If on same network

# Monitor ROS network
ros2 node list
ros2 topic list
ros2 topic echo /topic_name

# Sync data from Jetson
~/roboracer_project/shared/scripts/sync_from_jetson.sh

# Start training in background
cd ~/roboracer_project/phase4_learning
nohup python3 train.py > training.log 2>&1 &

# Monitor training
tail -f ~/roboracer_project/phase4_learning/training.log

# Upload results to server
scp -r results/ school-server:~/roboracer_project/shared/
```

---

## ✅ Verification Checklist

After setup, verify everything works:

- [ ] SSH key authentication works (no password prompt)
- [ ] Server directory structure created
- [ ] ROS 2 network configured (ros2 node list shows nodes)
- [ ] Data sync script works
- [ ] Isaac Lab installed and tested
- [ ] Backup script works
- [ ] Permissions set correctly
- [ ] Can connect from Jetson, desktop, and server

---

## 🎯 Next Steps

1. **Today:** Set up SSH keys and directory structure
2. **Tomorrow (Phase 1):** Use server for data archiving
3. **Week 4:** Install Isaac Lab on server
4. **Week 6:** Submit training jobs to server
5. **Week 14+:** Analyze and archive results

---

**Your school server is now the central hub for the entire roboracer project!**
