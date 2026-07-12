#!/usr/bin/env python3
"""
Records an MP4 of the ROOM world (test-plan variant): real env code, one env,
visual cuboids spawned for the walls/obstacles, goal disc, car marker.
A simple goal-seeking + gap-avoidance driver exercises the scene.

Run (server):
  export OMNI_KIT_ACCEPT_EULA=YES
  python roboracer_room_showcase.py --headless --enable_cameras \
      --seed 3 --steps 500 --out /tmp/room.mp4
"""

import argparse
import math
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--seed", type=int, default=3)
parser.add_argument("--steps", type=int, default=500)
parser.add_argument("--out", type=str, default="/tmp/roboracer_room.mp4")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np  # noqa: E402
import torch  # noqa: E402
import imageio  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.envs import ViewerCfg  # noqa: E402
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent / "envs"))
from roboracer_env import RoboracerEnv, RoboracerEnvCfg, generate_room  # noqa: E402


class RoomViewerEnv(RoboracerEnv):
    """One fixed room: first generation is kept, resets respawn in it."""

    _frozen = False

    def _regenerate_track(self, env_ids):
        if self._frozen:
            return
        super()._regenerate_track(env_ids)
        self._frozen = True


def yaw_quat(yaw):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def main():
    # pre-generate the same room the env will produce (same rng sequence)
    cfg = RoboracerEnvCfg()
    cfg.track_type = "room"
    cfg.scene.num_envs = 1
    preview_walls, _, preview_goal, sp_len = generate_room(
        np.random.default_rng(args_cli.seed), cfg)
    bb_min = preview_walls.reshape(-1, 2).min(axis=0)
    bb_max = preview_walls.reshape(-1, 2).max(axis=0)
    cx, cy = (bb_min + bb_max) / 2.0
    ext = float(max(bb_max - bb_min))
    cfg.viewer = ViewerCfg(
        eye=(float(cx) - 0.2 * ext, float(cy) - 0.35 * ext, 1.15 * ext + 2.0),
        lookat=(float(cx), float(cy), 0.0),
    )

    env = RoomViewerEnv(cfg, render_mode="rgb_array")
    env.rng = np.random.default_rng(args_cli.seed)  # reproduce the preview room
    env.reset()
    walls = env.walls[0].cpu().numpy()
    n_walls = int(env.wall_mask[0].sum())
    print(f"[Room] {n_walls} wall segments, shortest path {sp_len:.1f} m, "
          f"goal at {env.goal_xy[0].tolist()}")

    # visual geometry (analytic walls made visible)
    wall_mat = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.45, 0.48, 0.53), roughness=0.7)
    for i in range(n_walls):
        x1, y1, x2, y2 = walls[i]
        length = float(math.hypot(x2 - x1, y2 - y1))
        c = sim_utils.CuboidCfg(size=(max(length, 0.02), 0.04, 0.30), visual_material=wall_mat)
        c.func(f"/World/room/wall_{i}", c,
               translation=((x1 + x2) / 2, (y1 + y2) / 2, 0.15),
               orientation=yaw_quat(math.atan2(y2 - y1, x2 - x1)))
    goal_cfg = sim_utils.CylinderCfg(
        radius=float(cfg.goal_radius), height=0.02,
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.18, 0.62, 0.30)))
    gx, gy = env.goal_xy[0].tolist()
    goal_cfg.func("/World/room/goal", goal_cfg, translation=(gx, gy, 0.01))

    car_marker = VisualizationMarkers(VisualizationMarkersCfg(
        prim_path="/Visuals/car",
        markers={"car": sim_utils.CuboidCfg(
            size=(0.33, 0.24, 0.12),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.82, 0.19, 0.10)))},
    ))

    frames = []
    for i in range(args_cli.steps):
        obs = env._get_observations()["policy"]
        rays = obs[0, : cfg.num_rays] * cfg.lidar_max_range
        n = rays.shape[0]
        angles = torch.linspace(-135.0, 135.0, n, device=env.device)
        # goal-seek with gap avoidance
        sin_b, cos_b = float(obs[0, -2]), float(obs[0, -1])
        goal_bearing = math.degrees(math.atan2(sin_b, cos_b))
        front = (angles - goal_bearing).abs() <= 30.0
        blocked = float(rays[front].min()) < 0.9 if front.any() else True
        if blocked:
            open_dirs = (angles.abs() <= 90.0)
            masked = torch.where(open_dirs, rays, torch.zeros_like(rays))
            target = float(angles[masked.argmax()])
        else:
            target = goal_bearing
        steer = max(-1.0, min(1.0, target / 25.0))
        act = torch.tensor([[steer, -0.5]], device=env.device)  # ~0.25 throttle
        env.step(act)
        x, y, yaw = env.car_state[0, 0], env.car_state[0, 1], env.car_state[0, 2]
        car_marker.visualize(
            translations=torch.tensor([[float(x), float(y), 0.06]], device=env.device),
            orientations=torch.tensor([list(yaw_quat(float(yaw)))], device=env.device))
        f = env.render()
        if f is not None:
            frames.append(f)
        if (i + 1) % 100 == 0:
            print(f"[Room] step {i + 1}/{args_cli.steps} "
                  f"goals={env.total_goals} crashes={env.total_crashes}")

    print(f"[Room] writing {len(frames)} frames -> {args_cli.out}")
    with imageio.get_writer(args_cli.out, fps=20, quality=8) as w:
        for f in frames:
            w.append_data(f)
    print(f"[Room] done. goals={env.total_goals} crashes={env.total_crashes}")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
