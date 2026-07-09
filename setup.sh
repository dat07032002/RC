#!/bin/bash
# Roboracer Workspace Setup Script
# Run this once to initialize the workspace structure

set -e  # Exit on any error

echo "======================================================================"
echo "ROBORACER WORKSPACE SETUP"
echo "======================================================================"

# Check if in roboracer_ws
if [ ! -d "src" ]; then
    echo "Error: Not in roboracer_ws directory"
    echo "Run this script from: ~/roboracer_ws/"
    exit 1
fi

# Create directory structure
echo "Creating directory structure..."
mkdir -p config
mkdir -p scripts
mkdir -p launch
mkdir -p datasets/{isaac_lab,hardware}
mkdir -p models
mkdir -p logs
mkdir -p src/roboracer_config
mkdir -p src/roboracer_control/src
mkdir -p src/roboracer_perception/src
mkdir -p src/roboracer_safety/src

echo "✓ Directories created"

# Copy vehicle_params.yaml template if it doesn't exist
if [ ! -f "config/vehicle_params.yaml" ]; then
    echo "Creating config/vehicle_params.yaml template..."
    cat > config/vehicle_params.yaml << 'EOF'
# Roboracer Vehicle Parameters
# Fill in during Phase 1 System Identification

vehicle:
  wheelbase: 0.XXX
  track_width: 0.XXX
  mass: 2.0

steering:
  pwm_center: 1500
  pwm_left_max: 1000
  pwm_right_max: 2000
  angle_left_max: XXX
  angle_right_max: XXX
  servo_tau: 0.05
  servo_response_delay: XXX

throttle:
  pwm_center: 1500
  max_velocity: X.XX
  max_acceleration: X.XX
  friction_coefficient: -X.XX

sensors:
  lidar:
    model: "Hokuyo 10LX"
    frequency: 25
    noise_std_at_1m: 0.XXX

# See implementation_guide.md Phase 1 for complete template
EOF
    echo "✓ Config template created"
fi

# Create SSH config snippet
if ! grep -q "Host jetson" ~/.ssh/config 2>/dev/null; then
    echo ""
    echo "Add this to ~/.ssh/config for easy SSH access:"
    echo "---"
    echo "Host jetson"
    echo "    HostName 192.168.x.x  # REPLACE WITH YOUR JETSON IP"
    echo "    User jetson"
    echo "    Port 22"
    echo "---"
fi

# Make scripts executable
echo "Making scripts executable..."
chmod +x scripts/*.py 2>/dev/null || true

# Build workspace
echo ""
echo "Building ROS 2 workspace..."
colcon build --symlink-install 2>&1 | grep -E "Built|Failed|Error" || echo "Build in progress..."

echo ""
echo "======================================================================"
echo "SETUP COMPLETE!"
echo "======================================================================"
echo ""
echo "Next steps:"
echo "1. Update ~/.ssh/config with your Jetson IP"
echo "2. Test SSH: ssh jetson 'ros2 --version'"
echo "3. Source workspace: source install/setup.bash"
echo "4. Test scripts:"
echo "   - python3 scripts/servo_test.py"
echo "   - python3 scripts/throttle_test.py"
echo "   - python3 scripts/lidar_noise_test.py"
echo "5. Fill in vehicle_params.yaml during measurements"
echo ""
echo "Reference: ~/roboracer_ws/implementation_guide.md Phase 1"
echo "======================================================================"
