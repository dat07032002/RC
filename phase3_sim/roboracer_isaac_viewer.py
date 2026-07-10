#!/usr/bin/env python3
"""
Phase 3 viewer: renders the maze env as actual 3D geometry (MuJoCo-viewer style)
and records an MP4 of a gap-follower driving one fixed maze.

Unlike roboracer_isaaclab_task.py (headless training: walls are GPU tensors),
this spawns USD wall cuboids + a car marker so Isaac actually draws the scene.
One env, one fixed maze, overhead camera.

Run (server):
  cd ~/roboracer_project/phase3_sim
  export OMNI_KIT_ACCEPT_EULA=YES
  python roboracer_isaac_viewer.py --headless --enable_cameras \
      --steps 400 --out /tmp/roboracer_maze.mp4
"""

import argparse
import math
from pathlib import Path

import numpy as np

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Roboracer maze viewer/recorder")
parser.add_argument("--steps", type=int, default=400, help="control steps to record (20 Hz)")
parser.add_argument("--live", action="store_true",
                    help="run forever for livestream viewing (no video recording); "
                         "combine with --livestream 2")
parser.add_argument("--seed", type=int, default=7, help="maze seed")
parser.add_argument("--out", type=str, default="/tmp/roboracer_maze.mp4")
parser.add_argument(
    "--vehicle_params",
    type=str,
    default=str(Path.home() / "roboracer_project/phase1_sysid/config/vehicle_params.yaml"),
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch  # noqa: E402
import imageio  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg, ViewerCfg  # noqa: E402
from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg  # noqa: E402
from isaaclab.scene import InteractiveSceneCfg  # noqa: E402
from isaaclab.sim import SimulationCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402


# ---------------------------------------------------------------------------
# Maze generation — same rules as the training task (kept in sync manually
# until the env is refactored into an importable module).
# ---------------------------------------------------------------------------
def generate_corridor(rng: np.random.Generator):
    n_turns = int(rng.integers(3, 9))
    width = float(rng.uniform(0.9, 1.6))  # mid-range widths film better
    half = width / 2.0
    lo = max(1.6, width + 0.6)

    heading = 0.0
    pts = [np.zeros(2)]
    turns = []
    last_two = (0, 0)
    for i in range(n_turns + 1):
        seg_len = float(rng.uniform(lo, max(3.5, lo + 0.5)))
        d = np.array([math.cos(heading), math.sin(heading)])
        pts.append(pts[-1] + seg_len * d)
        if i < n_turns:
            t = int(rng.choice([-1, 1]))
            if last_two == (t, t):
                t = -t
            last_two = (last_two[1], t)
            turns.append(t)
            heading += t * math.pi / 2.0
    pts = np.array(pts)

    dirs = pts[1:] - pts[:-1]
    dirs = dirs / np.linalg.norm(dirs, axis=1, keepdims=True)
    normals = np.stack([-dirs[:, 1], dirs[:, 0]], axis=1)

    walls = []
    n_seg = len(dirs)
    for i in range(n_seg):
        for side, nrm in ((+1, normals[i]), (-1, -normals[i])):
            a = pts[i] + half * nrm
            b = pts[i + 1] + half * nrm
            if i > 0:
                inner = (turns[i - 1] == side)
                a = a + (half if inner else -half) * dirs[i]
            if i < n_seg - 1:
                inner = (turns[i] == side)
                b = b + (-half if inner else half) * dirs[i]
            walls.append([a[0], a[1], b[0], b[1]])
    n0, d0 = normals[0], dirs[0]
    walls.append([*(pts[0] - 0.3 * d0 + half * n0), *(pts[0] - 0.3 * d0 - half * n0)])
    ne = normals[-1]
    walls.append([*(pts[-1] + half * ne), *(pts[-1] - half * ne)])
    return np.array(walls, dtype=np.float32), pts.astype(np.float32), width


MAZE_WALLS, MAZE_CL, MAZE_WIDTH = generate_corridor(np.random.default_rng(args_cli.seed))
_bb_min = MAZE_WALLS.reshape(-1, 2).min(axis=0)
_bb_max = MAZE_WALLS.reshape(-1, 2).max(axis=0)
_center = (_bb_min + _bb_max) / 2.0
_extent = float(max(_bb_max - _bb_min))


def yaw_quat(yaw: float):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


@configclass
class ViewerEnvCfg(DirectRLEnvCfg):
    decimation = 5
    episode_length_s = 60.0
    action_space = 2
    observation_space = 64 + 4
    state_space = 0
    num_rays: int = 64

    sim: SimulationCfg = SimulationCfg(dt=0.01, render_interval=5)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=1, env_spacing=1.0)
    # overhead camera, slightly tilted so walls read as 3D
    viewer: ViewerCfg = ViewerCfg(
        eye=(float(_center[0]) - 0.25 * _extent, float(_center[1]) - 0.25 * _extent, 1.1 * _extent + 3.0),
        lookat=(float(_center[0]), float(_center[1]), 0.0),
    )

    # measured dynamics (matches training task defaults)
    wheelbase: float = 0.33
    steer_max_deg: float = 25.0
    steer_tau_s: float = 0.05
    v_max: float = 0.57
    a_max: float = 0.61
    car_radius: float = 0.14
    lidar_max_range: float = 10.0
    lidar_fov_deg: float = 270.0


class ViewerEnv(DirectRLEnv):
    cfg: ViewerEnvCfg

    def __init__(self, cfg, render_mode=None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, dev = self.num_envs, self.device
        self.car_state = torch.zeros(n, 5, device=dev)
        self._act = torch.zeros(n, 2, device=dev)
        self.walls = torch.as_tensor(MAZE_WALLS, device=dev).unsqueeze(0).repeat(n, 1, 1)
        fov = math.radians(cfg.lidar_fov_deg)
        self.ray_angles = torch.linspace(-fov / 2, fov / 2, cfg.num_rays, device=dev)
        self.steer_max_rad = math.radians(cfg.steer_max_deg)
        self.crash_count = 0

    def _setup_scene(self):
        spawn_ground = sim_utils.GroundPlaneCfg()
        spawn_ground.func("/World/ground", spawn_ground)
        light = sim_utils.DomeLightCfg(intensity=2500.0, color=(0.95, 0.95, 0.92))
        light.func("/World/Light", light)

        # walls as real cuboids
        wall_mat = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.42, 0.46, 0.52), roughness=0.7)
        for i, (x1, y1, x2, y2) in enumerate(MAZE_WALLS):
            length = float(math.hypot(x2 - x1, y2 - y1))
            yaw = math.atan2(y2 - y1, x2 - x1)
            cfg = sim_utils.CuboidCfg(size=(length, 0.05, 0.30), visual_material=wall_mat)
            cfg.func(
                f"/World/maze/wall_{i}", cfg,
                translation=((x1 + x2) / 2, (y1 + y2) / 2, 0.15),
                orientation=yaw_quat(yaw),
            )
        # goal disc
        goal_cfg = sim_utils.CylinderCfg(
            radius=0.25, height=0.02,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.18, 0.62, 0.30)),
        )
        gx, gy = MAZE_CL[-1]
        goal_cfg.func("/World/maze/goal", goal_cfg, translation=(float(gx), float(gy), 0.01))

        # car body as a visualization marker (pose updated every step)
        marker_cfg = VisualizationMarkersCfg(
            prim_path="/Visuals/car",
            markers={
                "car": sim_utils.CuboidCfg(
                    size=(0.33, 0.24, 0.12),
                    visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.82, 0.19, 0.10)),
                )
            },
        )
        self.car_marker = VisualizationMarkers(marker_cfg)

    # ---- same kinematics as the training task ----------------------------
    def _pre_physics_step(self, actions):
        self._act = actions.clamp(-1.0, 1.0)

    def _apply_action(self):
        steer_cmd = self._act[:, 0] * self.steer_max_rad
        throttle = (self._act[:, 1] + 1.0) / 2.0
        dt = self.cfg.sim.dt
        x, y, yaw, v, steer = self.car_state.unbind(dim=1)
        steer = steer + (steer_cmd - steer) * dt / self.cfg.steer_tau_s
        v_cmd = throttle * self.cfg.v_max
        dv = (v_cmd - v).clamp(-self.cfg.a_max * dt, self.cfg.a_max * dt)
        v = v + dv
        yaw = yaw + v / self.cfg.wheelbase * torch.tan(steer) * dt
        x = x + v * torch.cos(yaw) * dt
        y = y + v * torch.sin(yaw) * dt
        self.car_state = torch.stack([x, y, yaw, v, steer], dim=1)

    def _raycast(self):
        E, R = self.num_envs, self.cfg.num_rays
        pose = self.car_state
        ang = pose[:, 2:3] + self.ray_angles.unsqueeze(0)
        D = torch.stack([torch.cos(ang), torch.sin(ang)], -1).view(E, R, 1, 2)
        O = pose[:, :2].view(E, 1, 1, 2)
        P1 = self.walls[:, :, 0:2].view(E, 1, -1, 2)
        P2 = self.walls[:, :, 2:4].view(E, 1, -1, 2)
        e = P2 - P1
        AO = P1 - O
        denom = D[..., 0] * e[..., 1] - D[..., 1] * e[..., 0]
        t = (AO[..., 0] * e[..., 1] - AO[..., 1] * e[..., 0]) / denom
        u = (AO[..., 0] * D[..., 1] - AO[..., 1] * D[..., 0]) / -denom
        valid = (denom.abs() > 1e-8) & (t > 0) & (u >= 0) & (u <= 1)
        t = torch.where(valid, t, torch.full_like(t, self.cfg.lidar_max_range))
        return t.amin(dim=2).clamp(max=self.cfg.lidar_max_range)

    def _wall_distance(self):
        pos = self.car_state[:, :2].view(-1, 1, 2)
        a, b = self.walls[:, :, 0:2], self.walls[:, :, 2:4]
        ab = b - a
        t = ((pos - a) * ab).sum(-1) / ab.square().sum(-1).clamp_min(1e-9)
        proj = a + t.clamp(0, 1).unsqueeze(-1) * ab
        return (pos - proj).square().sum(-1).sqrt().amin(dim=1)

    def _get_observations(self):
        rays = self._raycast()
        obs = torch.cat(
            [rays / self.cfg.lidar_max_range,
             self.car_state[:, 3:4] / self.cfg.v_max,
             self.car_state[:, 4:5] / self.steer_max_rad,
             self._act],
            dim=1,
        )
        return {"policy": obs}

    def _get_rewards(self):
        return torch.zeros(self.num_envs, device=self.device)

    def _get_dones(self):
        crashed = self._wall_distance() < self.cfg.car_radius
        goal = torch.as_tensor(MAZE_CL[-1], device=self.device)
        reached = (self.car_state[:, :2] - goal).norm(dim=1) < 0.4
        self.crash_count += int(crashed.sum())
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        return crashed | reached, time_out

    def _reset_idx(self, env_ids):
        super()._reset_idx(env_ids)
        self.car_state[env_ids] = 0.0
        self.car_state[env_ids, 0] = 0.15

    def update_car_marker(self):
        x, y, yaw = self.car_state[0, 0], self.car_state[0, 1], self.car_state[0, 2]
        pos = torch.tensor([[float(x), float(y), 0.06]], device=self.device)
        q = yaw_quat(float(yaw))
        quat = torch.tensor([list(q)], device=self.device)
        self.car_marker.visualize(translations=pos, orientations=quat)


def gap_follower(obs: torch.Tensor, num_rays: int, fov_deg: float) -> torch.Tensor:
    """Steer toward the longest ray within +-60 deg of straight ahead."""
    rays = obs[:, :num_rays]
    n = rays.shape[1]
    angles = torch.linspace(-fov_deg / 2, fov_deg / 2, n, device=obs.device)
    front = (angles.abs() <= 60.0)
    masked = torch.where(front.unsqueeze(0), rays, torch.zeros_like(rays))
    best = masked.argmax(dim=1)
    target_ang = angles[best]                     # degrees
    steer = (target_ang / 25.0).clamp(-1.0, 1.0)  # -> normalized action
    # slow down when the closest front ray is near
    front_min = torch.where(front.unsqueeze(0), rays, torch.ones_like(rays)).min(dim=1).values
    throttle = torch.where(front_min * 10.0 < 0.8,
                           torch.full_like(front_min, 0.1),
                           torch.full_like(front_min, 0.8))
    return torch.stack([steer, throttle * 2 - 1], dim=1)


def main():
    cfg = ViewerEnvCfg()
    env = ViewerEnv(cfg, render_mode=None if args_cli.live else "rgb_array")
    obs, _ = env.reset()

    print(f"[Viewer] maze: {len(MAZE_WALLS)} walls, width {MAZE_WIDTH:.2f} m, "
          f"route {np.linalg.norm(np.diff(MAZE_CL, axis=0), axis=1).sum():.1f} m")

    if args_cli.live:
        print("[Viewer] LIVE mode: streaming until killed (connect WebRTC client)")
        i = 0
        while simulation_app.is_running():
            act = gap_follower(obs["policy"], cfg.num_rays, cfg.lidar_fov_deg)
            obs, _, terminated, truncated, _ = env.step(act)
            env.update_car_marker()
            i += 1
            if i % 2000 == 0:
                print(f"[Viewer] live step {i}, crashes={env.crash_count}")
        env.close()
        return

    frames = []
    for i in range(args_cli.steps):
        act = gap_follower(obs["policy"], cfg.num_rays, cfg.lidar_fov_deg)
        obs, _, terminated, truncated, _ = env.step(act)
        env.update_car_marker()
        frame = env.render()
        if frame is not None:
            frames.append(frame)
        if (i + 1) % 100 == 0:
            print(f"[Viewer] step {i + 1}/{args_cli.steps}, frames={len(frames)}")

    print(f"[Viewer] writing {len(frames)} frames -> {args_cli.out}")
    with imageio.get_writer(args_cli.out, fps=20, quality=7) as w:
        for f in frames:
            w.append_data(f)
    print(f"[Viewer] done. crashes={env.crash_count}")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
