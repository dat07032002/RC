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
    import math

    cfg = RoboracerPhysEnvCfg()
    cfg.scene.num_envs = 1
    # wide view: left circle (~R 0.8, center +y) and right circle (~R 1.1, -y)
    cfg.viewer.eye = (0.4, -4.2, 3.0)
    cfg.viewer.lookat = (0.4, -0.3, 0.0)
    env = ShowcaseEnv(cfg, render_mode="rgb_array")
    env.reset()

    # phases: 2 s straight | 8 s full LEFT circle | 8 s full RIGHT circle
    phases = [(40, 0.0, "straight"), (160, 1.0, "left"), (160, -1.0, "right")]
    frames = []
    step = 0
    for n_steps, steer, name in phases:
        yaw_acc, v_acc, cnt = 0.0, 0.0, 0
        yaw_prev = float(env.car_state[0, 2])
        for i in range(n_steps):
            act = torch.zeros(1, 2, device=env.device)
            act[:, 0] = steer
            act[:, 1] = 0.5  # moderate throttle so circles stay tight
            env.step(act)
            frame = env.render()
            if frame is not None:
                frames.append(frame)
            yaw = float(env.car_state[0, 2])
            dy = math.atan2(math.sin(yaw - yaw_prev), math.cos(yaw - yaw_prev))
            yaw_prev = yaw
            if i >= n_steps // 2:  # steady-state window
                yaw_acc += dy
                v_acc += float(env.car_state[0, 3])
                cnt += 1
            step += 1
        if steer != 0.0 and cnt:
            v = v_acc / cnt
            yr = yaw_acc / (cnt * 0.05)
            R = v / abs(yr) if abs(yr) > 1e-3 else float("inf")
            print(f"[Showcase] {name}: v={v:.2f} m/s, turn radius={R:.2f} m")

    print(f"[Showcase] writing {len(frames)} frames -> {args_cli.out}")
    with imageio.get_writer(args_cli.out, fps=20, quality=8) as w:
        for f in frames:
            w.append_data(f)
    print("[Showcase] done")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
