#!/usr/bin/env python3
"""
Phase 3 smoke test: launches Isaac and runs the maze env with random actions.
The env itself lives in envs/roboracer_env.py (importable, no CLI).

Run (on training server, conda env `isaaclab`):
  cd ~/roboracer_project/phase3_sim
  export OMNI_KIT_ACCEPT_EULA=YES
  python roboracer_isaaclab_task.py --headless --num_envs 64
"""

import argparse
import os
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Roboracer Phase 3 smoke test")
parser.add_argument("--num_envs", type=int, default=64, help="parallel environments")
parser.add_argument("--smoke_steps", type=int, default=200, help="random-action steps")
parser.add_argument("--track_type", type=str, default="corridor",
                    choices=["corridor", "room"], help="world variant")
parser.add_argument(
    "--vehicle_params",
    type=str,
    default=str(Path.home() / "roboracer_project/phase1_sysid/config/vehicle_params.yaml"),
    help="Phase 1 measured parameters (YAML)",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

os.environ["ROBORACER_VEHICLE_PARAMS"] = args_cli.vehicle_params

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# env module imports isaaclab.*, so it must come after the app is up
sys.path.insert(0, str(Path(__file__).resolve().parent / "envs"))
from roboracer_env import run_smoke  # noqa: E402

if __name__ == "__main__":
    run_smoke(args_cli.num_envs, args_cli.smoke_steps, args_cli.track_type)
    simulation_app.close()
