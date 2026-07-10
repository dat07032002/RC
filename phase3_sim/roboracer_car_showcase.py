#!/usr/bin/env python3
"""
Records a close-up MP4 of the physics car: 2 s straight run, then a full-lock
left circle (validated turn radius ~0.84 m keeps it in frame).

Run (server):
  export OMNI_KIT_ACCEPT_EULA=YES
  python roboracer_car_showcase.py --headless --enable_cameras --out /tmp/car.mp4
"""

import argparse
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--out", type=str, default="/tmp/roboracer_car.mp4")
parser.add_argument("--steps", type=int, default=360)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import imageio  # noqa: E402
import torch  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent / "envs"))
from roboracer_phys_env import RoboracerPhysEnv, RoboracerPhysEnvCfg  # noqa: E402


class ShowcaseEnv(RoboracerPhysEnv):
    """No terminations; open ground so the camera keeps the car in frame."""

    def _get_dones(self):
        self._sync_car_state()
        never = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self._min_wall_dist = torch.full((self.num_envs,), 1e9, device=self.device)
        self.crashed = never
        self.reached_goal = never
        return never, never


def main():
    cfg = RoboracerPhysEnvCfg()
    cfg.scene.num_envs = 1
    # circle center is ~(0, 0.84) once the car turns left from the origin
    cfg.viewer.eye = (2.2, -1.6, 1.4)
    cfg.viewer.lookat = (0.0, 0.8, 0.0)
    env = ShowcaseEnv(cfg, render_mode="rgb_array")
    env.reset()

    frames = []
    for i in range(args_cli.steps):
        act = torch.zeros(1, 2, device=env.device)
        act[:, 1] = 0.8                      # throttle
        act[:, 0] = 0.0 if i < 40 else 1.0   # straight 2 s, then full left
        env.step(act)
        frame = env.render()
        if frame is not None:
            frames.append(frame)
        if (i + 1) % 100 == 0:
            print(f"[Showcase] step {i + 1}/{args_cli.steps}")

    print(f"[Showcase] writing {len(frames)} frames -> {args_cli.out}")
    with imageio.get_writer(args_cli.out, fps=20, quality=8) as w:
        for f in frames:
            w.append_data(f)
    print("[Showcase] done")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
