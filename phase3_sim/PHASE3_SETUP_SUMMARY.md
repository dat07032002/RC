# Phase 3 Status: Isaac Lab (REAL) + Gymnasium Fallback

## ⚠️ Important clarification

The plan (comprehensive_plan.md) specifies **Isaac Lab** for Phase 3. Two things exist in this repo:

1. **`envs/roboracer_isaac_env.py`** — a **lightweight Gymnasium FALLBACK**, NOT Isaac Lab.
   Despite the filename, it is a simple kinematic toy model with placeholder parameters.
   Use it only for pipeline smoke-testing, never for final policy training.

2. **Real Isaac Lab** — installed on the training server at:
   `~/roboracer_project/isaac_lab_env/IsaacLab_official` (Isaac Sim 4.5 + Isaac Lab v2.1.0,
   conda env `isaaclab`, Python 3.10). This is the actual Phase 3 platform.

## Phase 3 correct workflow (Weeks 4-6, per plan)

1. Wait for Phase 1 measured parameters (`vehicle_params.yaml`)
2. Build the maze + Ackermann car task in Isaac Lab using MEASURED wheelbase,
   steering gain/lag, throttle curve, LiDAR noise model
3. Domain randomization: 20K trajectories (68% randomized mazes / 25% real track / 7% edge cases)
4. Output dataset feeds Phase 4 (Dreamer world model + PPO)

## Server usage

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate isaaclab
cd ~/roboracer_project/isaac_lab_env/IsaacLab_official
./isaaclab.sh -p <task_script.py> --headless
```
