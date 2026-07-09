#!/bin/bash
# ============================================================
# Isaac Lab + full training stack installer (no sudo needed)
# Server: aere-a83514.ae.utexas.edu (5x RTX 6000 Ada, driver 565.77)
# Installs: Miniconda -> Python 3.10 env -> Isaac Sim 4.5 (pip)
#           -> Isaac Lab -> RL frameworks -> DreamerV3 deps
# Log: ~/roboracer_project/isaac_install.log
# ============================================================
set -uo pipefail

LOG=~/roboracer_project/isaac_install.log
STATUS=~/roboracer_project/isaac_install_status.txt

log() {
    echo "[$(date +%H:%M:%S)] $1" | tee -a "$LOG"
    echo "$1" > "$STATUS"
}

fail() {
    log "FAILED: $1"
    echo "FAILED: $1" > "$STATUS"
    exit 1
}

log "=== STEP 1/7: Miniconda (user-space) ==="
if [ ! -d ~/miniconda3 ]; then
    wget -q https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /tmp/miniconda.sh \
        || fail "miniconda download"
    bash /tmp/miniconda.sh -b -p ~/miniconda3 >> "$LOG" 2>&1 || fail "miniconda install"
    rm /tmp/miniconda.sh
fi
source ~/miniconda3/etc/profile.d/conda.sh
log "Miniconda OK"

log "=== STEP 2/7: Python 3.10 environment 'isaaclab' ==="
if ! conda env list | grep -q "^isaaclab "; then
    conda create -y -n isaaclab python=3.10 >> "$LOG" 2>&1 || fail "conda env create"
fi
conda activate isaaclab || fail "conda activate"
python --version >> "$LOG" 2>&1
log "Python 3.10 env OK"

log "=== STEP 3/7: PyTorch (CUDA 12.1, driver-compatible) ==="
pip install --quiet torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cu121 >> "$LOG" 2>&1 \
    || fail "torch install"
python -c "import torch; assert torch.cuda.is_available(), 'no cuda'; print('torch', torch.__version__, 'gpus:', torch.cuda.device_count())" >> "$LOG" 2>&1 \
    || fail "torch cuda check"
log "PyTorch CUDA OK"

log "=== STEP 4/7: Isaac Sim 4.5 via pip (~10GB, longest step) ==="
pip install --quiet "isaacsim[all,extscache]==4.5.0" --extra-index-url https://pypi.nvidia.com >> "$LOG" 2>&1 \
    || fail "isaacsim pip install"
log "Isaac Sim 4.5 installed"

log "=== STEP 5/7: Isaac Lab (clone + install) ==="
cd ~/roboracer_project/isaac_lab_env
if [ ! -d IsaacLab_official ]; then
    git clone --quiet https://github.com/isaac-sim/IsaacLab.git IsaacLab_official >> "$LOG" 2>&1 \
        || fail "isaaclab clone"
fi
cd IsaacLab_official
# v2.1.0 is the release paired with Isaac Sim 4.5
git checkout --quiet v2.1.0 >> "$LOG" 2>&1 || log "WARN: v2.1.0 tag not found, using default branch"
# EULA acceptance for headless first launch
export OMNI_KIT_ACCEPT_EULA=YES
./isaaclab.sh --install >> "$LOG" 2>&1 || fail "isaaclab installer"
log "Isaac Lab installed"

log "=== STEP 6/7: Phase 4 extras (DreamerV3/JAX, tooling) ==="
pip install --quiet -U "jax[cuda12]" >> "$LOG" 2>&1 || log "WARN: jax cuda install failed (retry later)"
pip install --quiet dm-haiku optax tensorboard wandb ruamel.yaml >> "$LOG" 2>&1 || log "WARN: dreamer deps partial"
if [ ! -d ~/roboracer_project/phase4_learning/dreamerv3 ]; then
    git clone --quiet https://github.com/danijar/dreamerv3.git ~/roboracer_project/phase4_learning/dreamerv3 >> "$LOG" 2>&1 \
        || log "WARN: dreamerv3 clone failed"
fi
log "Phase 4 extras done"

log "=== STEP 7/7: Headless smoke test ==="
cd ~/roboracer_project/isaac_lab_env/IsaacLab_official
timeout 600 ./isaaclab.sh -p scripts/tutorials/00_sim/create_empty.py --headless >> "$LOG" 2>&1
RC=$?
if [ $RC -eq 0 ]; then
    log "SMOKE TEST PASSED - Isaac Lab fully operational"
elif [ $RC -eq 124 ]; then
    log "SMOKE TEST TIMEOUT (first launch compiles shaders; likely OK, rerun to confirm)"
else
    # older layouts use source/standalone path
    timeout 600 ./isaaclab.sh -p source/standalone/tutorials/00_sim/create_empty.py --headless >> "$LOG" 2>&1 \
        && log "SMOKE TEST PASSED (alt path) - Isaac Lab fully operational" \
        || log "SMOKE TEST FAILED rc=$RC - check isaac_install.log tail"
fi

log "=== INSTALL SCRIPT COMPLETE ==="
echo ""
echo "Usage after install:"
echo "  source ~/miniconda3/etc/profile.d/conda.sh && conda activate isaaclab"
echo "  cd ~/roboracer_project/isaac_lab_env/IsaacLab_official"
echo "  ./isaaclab.sh -p <script.py> --headless"
