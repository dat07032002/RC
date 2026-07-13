#!/usr/bin/env python3
"""
Phase 3: Roboracer maze-navigation env for Isaac Lab (DirectRLEnv) — importable module.

IMPORTANT: import this only AFTER AppLauncher has started the Omniverse app
(it imports isaaclab.* which requires a running app). Entry points:
  * phase3_sim/roboracer_isaaclab_task.py  — random-action smoke test
  * phase4_learning/train_isaac_ppo.py     — PPO+GRU training (rsl-rl)
Vehicle params path: env var ROBORACER_VEHICLE_PARAMS, else the default below.

STATUS: v2 — curriculum stage 3 (goal-conditioning + static obstacles).
  * Vehicle dynamics from Phase 1 measurements (vehicle_params.yaml, real schema)
  * Domain-randomized corridor mazes (width 0.5-2.0 m, 3-8 turns) per env
  * Planar LiDAR via batched ray-segment casting + measured noise model a+b*d
  * Collision + goal termination, progress/smoothness/clearance reward
  * Action delay buffer from latency budget
  * Goal-conditioned obs: distance + bearing to a lookahead waypoint 2 m ahead
    on the centerline (same interface the real car gets from map goal + localization)
  * 0-3 static box obstacles per track, hugging one wall with a guaranteed
    passable gap >= 0.5 m on the other side

Still TODO (Week 4+):
  [ ] Replace kinematic bicycle with USD Ackermann articulation (tire physics)
  [ ] Import Phase 2 real-track occupancy maps (25% of trajectory mix)
  [ ] Visual wall meshes for RViz/replay debugging (sim is currently analytic 2D)
"""

import math
import os
from pathlib import Path

import numpy as np
import yaml

import torch

import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.utils import configclass


# ---------------------------------------------------------------------------
# Phase 1 parameter loading — uses the REAL vehicle_params.yaml schema.
# Unmeasured entries hold the literal string "XXX"; _f() falls back per-key
# and prints what it did, so provisional values are visible at startup.
# ---------------------------------------------------------------------------
def _f(params: dict, dotted: str, default: float) -> float:
    v = params
    for k in dotted.split("."):
        v = v.get(k) if isinstance(v, dict) else None
    if isinstance(v, (int, float)) and not isinstance(v, bool):
        return float(v)
    print(f"[Phase3]   {dotted}: not measured -> default {default}")
    return float(default)


def load_vehicle_params(path: str) -> dict:
    p = Path(path)
    raw = {}
    if p.exists():
        with open(p) as f:
            raw = yaml.safe_load(f) or {}
        print(f"[Phase3] Loaded measured vehicle params: {p}")
    else:
        print("=" * 70)
        print("[Phase3] WARNING: vehicle_params.yaml not found — ALL values are")
        print("         defaults. NOT valid for sim-to-real transfer.")
        print("=" * 70)

    out = {
        "wheelbase": _f(raw, "vehicle.wheelbase", 0.33),
        "track_width": _f(raw, "vehicle.track_width", 0.24),
        # Full-lock circle measurements show meaningful left/right asymmetry.
        "steer_left_max_deg": _f(raw, "steering.angle_left_max", 24.93),
        "steer_right_max_deg": _f(raw, "steering.angle_right_max", 19.44),
        "steer_tau_s": _f(raw, "steering.servo_tau", 0.05),  # provisional until stand test
        # NOTE: measured on 6 m floor with speed cap — true max is higher (open test)
        "v_max": _f(raw, "throttle.max_velocity", 0.57),
        "a_max": _f(raw, "throttle.max_acceleration", 0.61),
        "throttle_tau_s": _f(raw, "throttle.throttle_response_tau", 0.1),
        # measured noise model: std = a + b * range
        "lidar_noise_a": _f(raw, "sensors.lidar.noise_std_coefficient_a", 0.003),
        "lidar_noise_b": _f(raw, "sensors.lidar.noise_std_coefficient_b", 0.0005),
        # UST-10LX guaranteed range is 10 m (yaml says 30 — that is the max
        # detectable, not guaranteed; indoors 10 m is the honest number)
        "lidar_max_range": 10.0,
        # sensor sits ahead of the rear axle (measured 0.295 m) — rays must
        # originate there, not at the axle, or cornering observations are wrong
        "lidar_x_offset": _f(raw, "sensors.lidar.x_offset", 0.0),
        # policy->motor latency unmeasured; 60 ms is a conservative placeholder,
        # DR range (20-100 ms) brackets it during training
        "latency_ms": _f(raw, "latency.total_latency_ms", 60.0),
    }
    print(f"[Phase3] params: {out}")
    return out


_DEFAULT_PARAMS = str(Path.home() / "roboracer_project/phase1_sysid/config/vehicle_params.yaml")
VEHICLE = load_vehicle_params(os.environ.get("ROBORACER_VEHICLE_PARAMS", _DEFAULT_PARAMS))


@configclass
class RoboracerEnvCfg(DirectRLEnvCfg):
    """Maze navigation. obs = LiDAR rays + v + steer + prev actions; act = [steer, throttle]."""

    # timing: 100 Hz physics, 20 Hz control (matches Jetson deployment rate)
    decimation = 5
    # 60 s: longest routes are ~31 m and v_max is 0.57 m/s -> ~55 s needed;
    # 30 s made many mazes unfinishable even for a perfect policy
    episode_length_s = 60.0

    num_rays: int = 64          # downsampled from 1081-beam UST-10LX, 270 deg FOV
    lidar_fov_deg: float = 270.0
    action_space = 2            # [steering, throttle]
    # rays + v + steer + prev_act(2) + goal[dist, sin(bearing), cos(bearing)]
    observation_space = num_rays + 7
    state_space = 0

    sim: SimulationCfg = SimulationCfg(dt=0.01, render_interval=5)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=64, env_spacing=6.0, replicate_physics=True
    )

    # --- measured dynamics (Phase 1) ---
    wheelbase: float = VEHICLE["wheelbase"]
    steer_left_max_deg: float = VEHICLE["steer_left_max_deg"]
    steer_right_max_deg: float = VEHICLE["steer_right_max_deg"]
    steer_max_deg: float = max(steer_left_max_deg, steer_right_max_deg)
    steer_tau_s: float = VEHICLE["steer_tau_s"]
    v_max: float = VEHICLE["v_max"]
    a_max: float = VEHICLE["a_max"]
    car_radius: float = 0.5 * VEHICLE["track_width"] + 0.02  # collision disc

    # --- sensor noise model (Phase 1): std = a + b * range ---
    lidar_noise_a: float = VEHICLE["lidar_noise_a"]
    lidar_noise_b: float = VEHICLE["lidar_noise_b"]
    lidar_max_range: float = VEHICLE["lidar_max_range"]
    lidar_x_offset: float = VEHICLE["lidar_x_offset"]  # ahead of rear axle

    # --- world type: "corridor" (original) or "room" (test-plan variant:
    # SLAM-mapped room, user-placed obstacles, random spawn + destination) ---
    track_type: str = "corridor"

    # --- room-mode DR (used when track_type == "room") ---
    dr_room_size = (3.0, 9.0)            # W, H each ~ U (adapt to any room)
    dr_room_obstacles = (0, 10)
    dr_room_obstacle_size = (0.15, 0.6)  # box side (m)
    room_goal_min_dist: float = 2.0      # goal at least this far from spawn
    room_clearance: float = 0.4          # spawn/goal clearance from geometry
    goal_radius: float = 0.3             # success = within this of the goal
    # PBRS discount. 1.0 = episodic-undiscounted form: standstill shaping is
    # exactly zero. (0.99 matched PPO's gamma but paid +0.01*k*d per idle step,
    # which out-earned goal completion over a 60 s episode -> measured 14%
    # loiter-timeouts in EMPTY rooms at the R1 gate. Empirical trap, fixed.)
    rew_gamma: float = 1.0

    # --- domain randomization (comprehensive_plan.md) ---
    dr_corridor_width = (0.5, 2.0)
    dr_num_turns = (3, 8)
    dr_seg_len = (1.6, 3.5)
    # obstacles are a primary randomization axis (real testbed = start/end
    # points with CHANGING track shape and obstacles between runs)
    dr_num_obstacles = (0, 4)          # boxes anywhere across the track width
    dr_obstacle_size = (0.12, 0.35)    # box side length (m)
    obstacle_min_gap: float = 0.50     # guaranteed passable gap on one side
    lookahead_m: float = 2.0           # goal waypoint distance along centerline

    # --- runtime DR (per-env, resampled each reset; SYSTEM_DESIGN section 7) ---
    dr_delay_steps = (1, 3)            # action delay 50-150 ms at 20 Hz control
    dr_speed_scale = (0.7, 1.3)        # v_max/a_max ±30% (capped-test uncertainty)
    dr_noise_mult = (1.0, 10.0)        # x measured lidar noise (up to ~5 cm @ 5 m)
    # --- structured estimator-error model (goal obs = EKF/SLAM output proxy).
    # Real estimator error is temporally CORRELATED (drift), not white: model
    # it as an OU random walk per env, plus small white jitter, plus rare
    # relocalization jumps that snap the drift toward zero.
    dr_goal_bearing_noise = (0.005, 0.03)   # rad std, white component
    dr_goal_dist_noise = 0.02               # relative std, white component
    dr_goal_drift = (0.004, 0.03)           # OU step size (stationary std ~5x)
    goal_drift_pull = 0.02                  # OU mean reversion per step
    reloc_jump_prob = 0.003                 # ~1 correction per 17 s per env

    # rewards
    rew_progress: float = 4.0
    rew_smoothness: float = 0.05
    rew_crash: float = -10.0
    rew_goal: float = 10.0
    rew_clearance: float = 1.0         # penalty slope inside clearance_dist
    clearance_dist: float = 0.25       # start penalizing below this wall distance

    max_wall_segments: int = 48        # corridor: 24+obstacles; room: 4+10x4
    max_centerline_pts: int = 12


# ---------------------------------------------------------------------------
# Maze generation: corridor with random 90-deg turns.
# Returns wall segments [S,4], centerline points [P,2].
# ---------------------------------------------------------------------------
def generate_corridor(rng: np.random.Generator, cfg: RoboracerEnvCfg):
    n_turns = int(rng.integers(cfg.dr_num_turns[0], cfg.dr_num_turns[1] + 1))
    width = float(rng.uniform(*cfg.dr_corridor_width))
    half = width / 2.0

    # centerline: keep segments longer than the corridor is wide so offset
    # walls join cleanly at corners
    lo = max(cfg.dr_seg_len[0], width + 0.6)
    heading = 0.0
    pts = [np.zeros(2)]
    turns = []
    last_two = (0, 0)
    for i in range(n_turns + 1):
        seg_len = float(rng.uniform(lo, max(cfg.dr_seg_len[1], lo + 0.5)))
        d = np.array([math.cos(heading), math.sin(heading)])
        pts.append(pts[-1] + seg_len * d)
        if i < n_turns:
            t = int(rng.choice([-1, 1]))
            if last_two == (t, t):  # 3 same turns in a row would loop back
                t = -t
            last_two = (last_two[1], t)
            turns.append(t)
            heading += t * math.pi / 2.0
    pts = np.array(pts)  # [P,2]

    dirs = pts[1:] - pts[:-1]
    dirs = dirs / np.linalg.norm(dirs, axis=1, keepdims=True)
    normals = np.stack([-dirs[:, 1], dirs[:, 0]], axis=1)  # left normal

    walls = []
    n_seg = len(dirs)
    for i in range(n_seg):
        for side, nrm in ((+1, normals[i]), (-1, -normals[i])):
            a = pts[i] + half * nrm
            b = pts[i + 1] + half * nrm
            # corner joins: inner wall shortens by half-width, outer extends
            if i > 0:
                inner = (turns[i - 1] == side)  # left turn -> left wall inner
                a = a + (half if inner else -half) * dirs[i]
            if i < n_seg - 1:
                inner = (turns[i] == side)
                b = b + (-half if inner else half) * dirs[i]
            walls.append([a[0], a[1], b[0], b[1]])
    # start cap (behind spawn) and end cap
    n0, d0 = normals[0], dirs[0]
    walls.append([*(pts[0] - 0.3 * d0 + half * n0), *(pts[0] - 0.3 * d0 - half * n0)])
    ne, de = normals[-1], dirs[-1]
    walls.append([*(pts[-1] + half * ne), *(pts[-1] - half * ne)])

    # --- stage 3: static box obstacles hugging one wall, gap guaranteed ------
    n_obs = int(rng.integers(cfg.dr_num_obstacles[0], cfg.dr_num_obstacles[1] + 1))
    seg_lens = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    for _ in range(n_obs):
        c = float(rng.uniform(*cfg.dr_obstacle_size))
        if c > width - cfg.obstacle_min_gap - 0.1:
            continue  # corridor too narrow to leave a passable gap
        cand = [i for i in range(n_seg) if seg_lens[i] >= 1.4]
        if not cand:
            continue
        i = int(rng.choice(cand))
        u_lo = 1.0 if i == 0 else 0.5   # keep the spawn area clear
        u = float(rng.uniform(u_lo, seg_lens[i] - 0.5))
        # lateral placement: anywhere from wall-hugging to mid-track, with the
        # passable gap >= min_gap guaranteed on the chosen side
        side = int(rng.choice([-1, 1]))
        off_lo = -(half - c / 2 - 0.05)                 # against the far wall
        off_hi = half - cfg.obstacle_min_gap - c / 2    # gap-side limit
        off = side * float(rng.uniform(off_lo, off_hi))
        ctr = pts[i] + u * dirs[i] + off * normals[i]
        d, nv = dirs[i], normals[i]
        corners = [
            ctr + (c / 2) * d + (c / 2) * nv,
            ctr + (c / 2) * d - (c / 2) * nv,
            ctr - (c / 2) * d - (c / 2) * nv,
            ctr - (c / 2) * d + (c / 2) * nv,
        ]
        for k in range(4):
            a, b = corners[k], corners[(k + 1) % 4]
            walls.append([a[0], a[1], b[0], b[1]])

    return np.array(walls, dtype=np.float32), pts.astype(np.float32)


# ---------------------------------------------------------------------------
# Room generation (test-plan variant): rectangular room, scattered box
# obstacles, random spawn pose + goal with guaranteed reachability.
# Returns walls [S,4], spawn [x,y,yaw], goal [x,y], shortest_path_len (m).
# ---------------------------------------------------------------------------
def _room_flood_fill(W, H, obstacles, start, goal, res=0.2, inflate=0.40):
    # inflate=0.40 -> certified paths are >=0.8 m wide. The flood fill is a
    # HOLONOMIC check; the car has R_min=0.71 m and cannot thread the 0.5 m
    # S-gaps a 0.25 inflation certifies (measured: R2 success ceiling ~60%
    # partly from kinematically-unwinnable worlds).
    """BFS on a coarse grid; returns shortest path length in meters or None."""
    nx, ny = max(int(W / res), 2), max(int(H / res), 2)
    occ = np.zeros((nx, ny), dtype=bool)
    for cx, cy, c in obstacles:
        r = c / 2 + inflate
        x0, x1 = max(int((cx - r) / res), 0), min(int((cx + r) / res) + 1, nx)
        y0, y1 = max(int((cy - r) / res), 0), min(int((cy + r) / res) + 1, ny)
        occ[x0:x1, y0:y1] = True
    # inflate room walls
    b = max(int(inflate / res), 1)
    occ[:b, :] = True; occ[-b:, :] = True; occ[:, :b] = True; occ[:, -b:] = True

    def cell(p):
        return (min(max(int(p[0] / res), 0), nx - 1), min(max(int(p[1] / res), 0), ny - 1))

    s, g = cell(start), cell(goal)
    if occ[s] or occ[g]:
        return None
    dist = np.full((nx, ny), -1.0)
    dist[s] = 0.0
    frontier = [s]
    diag = res * math.sqrt(2.0)
    while frontier:
        nxt = []
        for (i, j) in frontier:
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1),
                           (1, 1), (1, -1), (-1, 1), (-1, -1)):
                ii, jj = i + di, j + dj
                if 0 <= ii < nx and 0 <= jj < ny and not occ[ii, jj] and dist[ii, jj] < 0:
                    dist[ii, jj] = dist[i, j] + (diag if di and dj else res)
                    if (ii, jj) == g:
                        return float(dist[ii, jj])
                    nxt.append((ii, jj))
        frontier = nxt
    return None


def generate_room(rng: np.random.Generator, cfg: RoboracerEnvCfg):
    W = float(rng.uniform(*cfg.dr_room_size))
    H = float(rng.uniform(*cfg.dr_room_size))
    clr = cfg.room_clearance
    n_obs = int(rng.integers(cfg.dr_room_obstacles[0], cfg.dr_room_obstacles[1] + 1))

    for attempt in range(12):
        obstacles = []
        for _ in range(n_obs):
            c = float(rng.uniform(*cfg.dr_room_obstacle_size))
            obstacles.append((
                float(rng.uniform(clr + c / 2, W - clr - c / 2)),
                float(rng.uniform(clr + c / 2, H - clr - c / 2)),
                c,
            ))

        def clear(p):
            return (clr <= p[0] <= W - clr and clr <= p[1] <= H - clr
                    and all(max(abs(p[0] - ox), abs(p[1] - oy)) > c / 2 + clr
                            for ox, oy, c in obstacles))

        def free_point():
            for _ in range(50):
                p = (float(rng.uniform(clr, W - clr)), float(rng.uniform(clr, H - clr)))
                if clear(p):
                    return p
            return None

        def free_pose():
            # collision is checked at BOTH axles: the front point (wheelbase
            # ahead along heading) must also be clear or the car spawns crashed
            for _ in range(50):
                p = free_point()
                if p is None:
                    return None
                yaw = float(rng.uniform(-math.pi, math.pi))
                front = (p[0] + cfg.wheelbase * math.cos(yaw),
                         p[1] + cfg.wheelbase * math.sin(yaw))
                if clear(front):
                    return (p[0], p[1], yaw)
            return None

        pose = free_pose()
        goal = free_point()
        min_d = min(cfg.room_goal_min_dist, 0.5 * math.hypot(W, H))
        if pose is None or goal is None or math.hypot(
                goal[0] - pose[0], goal[1] - pose[1]) < min_d:
            continue
        spawn = (pose[0], pose[1])
        sp_len = _room_flood_fill(W, H, obstacles, spawn, goal)
        if sp_len is None:
            if attempt >= 5 and n_obs > 0:
                n_obs -= 1  # too cluttered — thin out and retry
            continue

        walls = [[0, 0, W, 0], [W, 0, W, H], [W, H, 0, H], [0, H, 0, 0]]
        for ox, oy, c in obstacles:
            h = c / 2
            corners = [(ox - h, oy - h), (ox + h, oy - h), (ox + h, oy + h), (ox - h, oy + h)]
            for k in range(4):
                a, b2 = corners[k], corners[(k + 1) % 4]
                walls.append([a[0], a[1], b2[0], b2[1]])
        return (np.array(walls, dtype=np.float32),
                np.array([pose[0], pose[1], pose[2]], dtype=np.float32),
                np.array(goal, dtype=np.float32),
                sp_len)

    # degenerate fallback: empty room, straight-line task (never expected)
    walls = np.array([[0, 0, W, 0], [W, 0, W, H], [W, H, 0, H], [0, H, 0, 0]],
                     dtype=np.float32)
    spawn = np.array([clr + 0.1, clr + 0.1, 0.0], dtype=np.float32)
    goal = np.array([W - clr - 0.1, H - clr - 0.1], dtype=np.float32)
    return walls, spawn, goal, float(np.linalg.norm(goal - spawn[:2]))


class RoboracerEnv(DirectRLEnv):
    """Maze navigation: kinematic bicycle car + analytic planar LiDAR.

    Track walls live as 2D segments on the GPU; LiDAR is batched ray-segment
    intersection. No USD car asset yet — dynamics are the measured bicycle
    model, which is the honest fidelity level Phase 1 supports so far.
    """

    cfg: RoboracerEnvCfg

    def __init__(self, cfg: RoboracerEnvCfg, render_mode=None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, dev = self.num_envs, self.device
        self.rng = np.random.default_rng(42)

        # bicycle-model state: [x, y, yaw, v, steer_angle]
        self.car_state = torch.zeros(n, 5, device=dev)
        self.prev_actions = torch.zeros(n, 2, device=dev)
        self._act = torch.zeros(n, 2, device=dev)

        # action delay buffer sized for the max DR delay; per-env delay index
        self.action_buf = torch.zeros(cfg.dr_delay_steps[1] + 1, n, 2, device=dev)
        self.delay_e = torch.full((n,), cfg.dr_delay_steps[0], dtype=torch.long, device=dev)
        # per-env dynamics/sensor randomization (resampled in _reset_idx)
        self.vmax_e = torch.full((n,), cfg.v_max, device=dev)
        self.amax_e = torch.full((n,), cfg.a_max, device=dev)
        self.noise_mult_e = torch.ones(n, device=dev)
        self.bearing_noise_e = torch.zeros(n, device=dev)
        # estimator drift state (OU): correlated bias on goal bearing/distance
        self.drift_e = torch.zeros(n, device=dev)          # per-env OU step size
        self.bearing_bias = torch.zeros(n, device=dev)     # rad
        self.dist_bias = torch.zeros(n, device=dev)        # relative

        # track storage (padded)
        S, P = cfg.max_wall_segments, cfg.max_centerline_pts
        self.walls = torch.zeros(n, S, 4, device=dev)
        self.wall_mask = torch.zeros(n, S, dtype=torch.bool, device=dev)
        self.cl_pts = torch.zeros(n, P, 2, device=dev)
        self.cl_mask = torch.zeros(n, P - 1, dtype=torch.bool, device=dev)  # segment mask
        self.cl_cum = torch.zeros(n, P - 1, device=dev)  # arc length at segment start
        self.total_len = torch.ones(n, device=dev)
        self.progress = torch.zeros(n, device=dev)
        # goal in world (room) / centerline-end (corridor); one obs code path
        self.goal_xy = torch.zeros(n, 2, device=dev)
        self.spawn_pose = torch.zeros(n, 3, device=dev)
        self.shortest_len = torch.ones(n, device=dev)   # flood-fill path, for SPL
        self.prev_goal_dist = torch.zeros(n, device=dev)  # PBRS potential state
        self._min_wall_dist = torch.full((n,), 1e9, device=dev)
        self.crashed = torch.zeros(n, dtype=torch.bool, device=dev)
        self.reached_goal = torch.zeros(n, dtype=torch.bool, device=dev)
        # cumulative episode-outcome counters (survive per-env resets)
        self.total_crashes = 0
        self.total_goals = 0

        # precomputed LiDAR ray angles (car frame), 270 deg FOV
        fov = math.radians(cfg.lidar_fov_deg)
        self.ray_angles = torch.linspace(-fov / 2, fov / 2, cfg.num_rays, device=dev)
        self.steer_max_rad = math.radians(cfg.steer_max_deg)
        self.steer_left_max_rad = math.radians(cfg.steer_left_max_deg)
        self.steer_right_max_rad = math.radians(cfg.steer_right_max_deg)

    # ---- scene ----------------------------------------------------------
    def _setup_scene(self):
        spawn_ground = sim_utils.GroundPlaneCfg()
        spawn_ground.func("/World/ground", spawn_ground)
        spawn_light = sim_utils.DomeLightCfg(intensity=2000.0)
        spawn_light.func("/World/Light", spawn_light)

    # ---- track management ------------------------------------------------
    def _regenerate_track(self, env_ids):
        if self.cfg.track_type == "room":
            self._regenerate_room(env_ids)
            return
        cfg = self.cfg
        for e in env_ids.tolist():
            walls, cl = generate_corridor(self.rng, cfg)
            s = min(len(walls), cfg.max_wall_segments)
            self.walls[e].zero_()
            self.wall_mask[e].zero_()
            self.walls[e, :s] = torch.as_tensor(walls[:s], device=self.device)
            self.wall_mask[e, :s] = True

            p = min(len(cl), cfg.max_centerline_pts)
            self.cl_pts[e].zero_()
            self.cl_mask[e].zero_()
            self.cl_pts[e, :p] = torch.as_tensor(cl[:p], device=self.device)
            seg_len = np.linalg.norm(np.diff(cl[:p], axis=0), axis=1)
            cum = np.concatenate([[0.0], np.cumsum(seg_len)])
            self.cl_cum[e, : p - 1] = torch.as_tensor(cum[:-1], dtype=torch.float32, device=self.device)
            self.cl_mask[e, : p - 1] = True
            self.total_len[e] = float(cum[-1])
            # unified goal/spawn representation
            self.goal_xy[e] = torch.as_tensor(cl[p - 1], device=self.device)
            self.spawn_pose[e] = torch.tensor([0.15, 0.0, 0.0], device=self.device)
            self.shortest_len[e] = float(cum[-1])

    def _regenerate_room(self, env_ids):
        cfg = self.cfg
        for e in env_ids.tolist():
            walls, spawn, goal, sp_len = generate_room(self.rng, cfg)
            s = min(len(walls), cfg.max_wall_segments)
            self.walls[e].zero_()
            self.wall_mask[e].zero_()
            self.walls[e, :s] = torch.as_tensor(walls[:s], device=self.device)
            self.wall_mask[e, :s] = True
            # no centerline in room mode
            self.cl_pts[e].zero_()
            self.cl_mask[e].zero_()
            self.cl_cum[e].zero_()
            self.total_len[e] = 1.0
            self.goal_xy[e] = torch.as_tensor(goal, device=self.device)
            self.spawn_pose[e] = torch.as_tensor(spawn, device=self.device)
            self.shortest_len[e] = sp_len

    def _arc_progress(self) -> torch.Tensor:
        """Project car position onto centerline -> arc length travelled."""
        pos = self.car_state[:, :2].unsqueeze(1)               # [E,1,2]
        a = self.cl_pts[:, :-1]                                # [E,P-1,2]
        b = self.cl_pts[:, 1:]
        ab = b - a
        ab_len2 = ab.square().sum(-1).clamp_min(1e-9)
        t = ((pos - a) * ab).sum(-1) / ab_len2
        t = t.clamp(0.0, 1.0)
        proj = a + t.unsqueeze(-1) * ab
        d2 = (pos - proj).square().sum(-1)
        d2 = torch.where(self.cl_mask, d2, torch.full_like(d2, 1e9))
        idx = d2.argmin(dim=1)                                  # nearest segment
        ar = torch.arange(self.num_envs, device=self.device)
        seg_len = ab_len2.sqrt()
        return self.cl_cum[ar, idx] + t[ar, idx] * seg_len[ar, idx]

    def _lookahead_waypoint(self) -> torch.Tensor:
        """Point on the centerline `lookahead_m` ahead of current progress: [E,2].

        This is the training-time stand-in for the deployed global planner's
        lookahead waypoint (map-frame goal -> route -> waypoint ~2 m ahead)."""
        s_look = torch.minimum(self.progress + self.cfg.lookahead_m, self.total_len - 0.05)
        cum = torch.where(self.cl_mask, self.cl_cum, torch.full_like(self.cl_cum, 1e9))
        idx = ((s_look.unsqueeze(1) >= cum) & self.cl_mask).sum(dim=1) - 1
        idx = idx.clamp(min=0)
        ar = torch.arange(self.num_envs, device=self.device)
        a = self.cl_pts[ar, idx]
        b = self.cl_pts[ar, idx + 1]
        seg = b - a
        seg_len = seg.norm(dim=1).clamp_min(1e-6)
        t = ((s_look - self.cl_cum[ar, idx]) / seg_len).clamp(0.0, 1.0)
        return a + t.unsqueeze(1) * seg

    # ---- MDP ------------------------------------------------------------
    def _pre_physics_step(self, actions: torch.Tensor):
        self.action_buf = torch.roll(self.action_buf, shifts=-1, dims=0)
        self.action_buf[-1] = actions.clamp(-1.0, 1.0)
        # per-env delay: env e acts on the command issued delay_e steps ago
        D = self.action_buf.shape[0]
        ar = torch.arange(self.num_envs, device=self.device)
        self._act = self.action_buf[D - 1 - self.delay_e, ar]

    def _apply_action(self):
        steer_action = self._act[:, 0]
        steer_cmd = torch.where(
            steer_action >= 0.0,
            steer_action * self.steer_left_max_rad,
            steer_action * self.steer_right_max_rad,
        )
        throttle = (self._act[:, 1] + 1.0) / 2.0  # [-1,1] -> [0,1]

        dt = self.cfg.sim.dt
        x, y, yaw, v, steer = self.car_state.unbind(dim=1)
        # first-order steering lag (measured tau)
        steer = steer + (steer_cmd - steer) * dt / self.cfg.steer_tau_s
        # longitudinal: accel-limited approach to commanded speed (per-env DR)
        v_cmd = throttle * self.vmax_e
        a_lim = self.amax_e * dt
        dv = torch.clamp(v_cmd - v, -a_lim, a_lim)
        v = v + dv
        # bicycle kinematics with measured wheelbase
        yaw = yaw + v / self.cfg.wheelbase * torch.tan(steer) * dt
        x = x + v * torch.cos(yaw) * dt
        y = y + v * torch.sin(yaw) * dt
        self.car_state = torch.stack([x, y, yaw, v, steer], dim=1)

    def _raycast_lidar(self) -> torch.Tensor:
        """Batched ray-segment intersection: [E, R] ranges."""
        E, R = self.num_envs, self.cfg.num_rays
        pose = self.car_state
        ang = pose[:, 2:3] + self.ray_angles.unsqueeze(0)       # [E,R]
        D = torch.stack([torch.cos(ang), torch.sin(ang)], -1)   # [E,R,2]
        # rays originate at the sensor (0.295 m ahead of rear axle, measured)
        fwd = torch.stack([torch.cos(pose[:, 2]), torch.sin(pose[:, 2])], -1)
        O = (pose[:, :2] + self.cfg.lidar_x_offset * fwd).view(E, 1, 1, 2)
        D = D.view(E, R, 1, 2)
        P1 = self.walls[:, :, 0:2].view(E, 1, -1, 2)
        P2 = self.walls[:, :, 2:4].view(E, 1, -1, 2)
        e = P2 - P1
        AO = P1 - O
        denom = D[..., 0] * e[..., 1] - D[..., 1] * e[..., 0]   # [E,R,S]
        t = (AO[..., 0] * e[..., 1] - AO[..., 1] * e[..., 0]) / denom
        u = (AO[..., 0] * D[..., 1] - AO[..., 1] * D[..., 0]) / -denom
        valid = (denom.abs() > 1e-8) & (t > 0) & (u >= 0) & (u <= 1)
        valid = valid & self.wall_mask.view(E, 1, -1)
        t = torch.where(valid, t, torch.full_like(t, self.cfg.lidar_max_range))
        return t.amin(dim=2).clamp(max=self.cfg.lidar_max_range)

    def _wall_distance(self) -> torch.Tensor:
        """Min distance from the car body to any wall segment: [E].

        Two collision discs — rear axle and front axle — so nose-first
        impacts are detected, not just body-center proximity."""
        yaw = self.car_state[:, 2]
        fwd = torch.stack([torch.cos(yaw), torch.sin(yaw)], -1)
        rear = self.car_state[:, :2]
        front = rear + self.cfg.wheelbase * fwd
        pos = torch.stack([rear, front], dim=1).view(-1, 2, 1, 2)   # [E,2,1,2]
        a = self.walls[:, :, 0:2].unsqueeze(1)                       # [E,1,S,2]
        b = self.walls[:, :, 2:4].unsqueeze(1)
        ab = b - a
        t = ((pos - a) * ab).sum(-1) / ab.square().sum(-1).clamp_min(1e-9)
        proj = a + t.clamp(0, 1).unsqueeze(-1) * ab
        d = (pos - proj).square().sum(-1).sqrt()                     # [E,2,S]
        d = torch.where(self.wall_mask.unsqueeze(1), d, torch.full_like(d, 1e9))
        return d.amin(dim=(1, 2))

    def _get_observations(self) -> dict:
        rays = self._raycast_lidar()
        # measured noise model x per-env DR multiplier (measured values are the
        # sensor's best case; real maze surfaces/angles are worse)
        noise_std = (self.cfg.lidar_noise_a + self.cfg.lidar_noise_b * rays) \
            * self.noise_mult_e.unsqueeze(1)
        rays = (rays + torch.randn_like(rays) * noise_std).clamp_min(0.0)
        # goal conditioning: bearing/distance to the FINAL goal (world frame).
        # Deliberately NOT a route waypoint: at deployment only the destination
        # (in the SLAM map) + localization exist. The policy must follow
        # geometry from LiDAR and use goal direction as guidance.
        rel = self.goal_xy - self.car_state[:, :2]
        dist = rel.norm(dim=1, keepdim=True)
        bearing = torch.atan2(rel[:, 1], rel[:, 0]) - self.car_state[:, 2]
        # estimator-error model: OU drift (correlated) + white jitter + jumps.
        # Called once per control step, so this advances the drift at 20 Hz.
        lam = self.cfg.goal_drift_pull
        self.bearing_bias = (1 - lam) * self.bearing_bias \
            + torch.randn_like(self.bearing_bias) * self.drift_e
        self.dist_bias = (1 - lam) * self.dist_bias \
            + torch.randn_like(self.dist_bias) * self.drift_e * 0.5
        # relocalization events: scan-match correction snaps drift toward zero
        jump = torch.rand_like(self.bearing_bias) < self.cfg.reloc_jump_prob
        keep = torch.rand_like(self.bearing_bias) * 0.3
        self.bearing_bias = torch.where(jump, self.bearing_bias * keep, self.bearing_bias)
        self.dist_bias = torch.where(jump, self.dist_bias * keep, self.dist_bias)

        bearing = bearing + self.bearing_bias \
            + torch.randn_like(bearing) * self.bearing_noise_e
        dist = dist * (1.0 + self.dist_bias.unsqueeze(1)
                       + torch.randn_like(dist) * self.cfg.dr_goal_dist_noise)
        obs = torch.cat(
            [
                rays / self.cfg.lidar_max_range,
                self.car_state[:, 3:4] / self.cfg.v_max,
                self.car_state[:, 4:5] / torch.where(
                    self.car_state[:, 4:5] >= 0.0,
                    self.steer_left_max_rad,
                    self.steer_right_max_rad,
                ),
                self.prev_actions,
                (dist / self.cfg.lidar_max_range).clamp(max=2.0),
                torch.sin(bearing).unsqueeze(1),
                torch.cos(bearing).unsqueeze(1),
            ],
            dim=1,
        )
        self.prev_actions = self._act.clone()
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        if self.cfg.track_type == "room":
            # PBRS (Ng et al. 1999): gamma*Phi(s') - Phi(s), Phi = -k*dist.
            # Policy-invariant dense shaping — cannot induce circling exploits.
            d = (self.goal_xy - self.car_state[:, :2]).norm(dim=1)
            shaping = self.cfg.rew_progress * (
                self.prev_goal_dist - self.cfg.rew_gamma * d)
            self.prev_goal_dist = d
            ds = torch.zeros_like(d)  # corridor arc-progress unused
        else:
            s = self._arc_progress()
            ds = s - self.progress
            self.progress = s
            shaping = self.cfg.rew_progress * ds
        smoothness = (self._act - self.prev_actions).square().sum(dim=1)
        rew = shaping - self.cfg.rew_smoothness * smoothness
        # clearance shaping: linear penalty once closer than clearance_dist
        rew = rew - self.cfg.rew_clearance * (
            self.cfg.clearance_dist - self._min_wall_dist
        ).clamp(min=0.0)
        rew = rew + self.cfg.rew_crash * self.crashed.float()
        rew = rew + self.cfg.rew_goal * self.reached_goal.float()
        return rew

    def _get_dones(self):
        self._min_wall_dist = self._wall_distance()
        self.crashed = self._min_wall_dist < self.cfg.car_radius
        if self.cfg.track_type == "room":
            goal_dist = (self.goal_xy - self.car_state[:, :2]).norm(dim=1)
            self.reached_goal = goal_dist < self.cfg.goal_radius
        else:
            self.reached_goal = self.progress > (self.total_len - 0.4)
        self.total_crashes += int(self.crashed.sum())
        self.total_goals += int(self.reached_goal.sum())
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        return self.crashed | self.reached_goal, time_out

    def _reset_idx(self, env_ids):
        super()._reset_idx(env_ids)
        self._regenerate_track(env_ids)
        self.car_state[env_ids] = 0.0
        k = len(env_ids)
        # spawn at the generated pose (+ small noise; room yaw is random already)
        self.car_state[env_ids, 0:3] = self.spawn_pose[env_ids]
        if self.cfg.track_type != "room":
            self.car_state[env_ids, 1] += 0.1 * (torch.rand(k, device=self.device) - 0.5)
            self.car_state[env_ids, 2] += 0.1 * (torch.rand(k, device=self.device) - 0.5)
        self.prev_goal_dist[env_ids] = (
            self.goal_xy[env_ids] - self.car_state[env_ids, :2]).norm(dim=1)
        self.action_buf[:, env_ids] = 0.0
        self.prev_actions[env_ids] = 0.0
        self.progress[env_ids] = 0.0
        self.crashed[env_ids] = False
        self.reached_goal[env_ids] = False
        # resample per-env DR (delay, dynamics scale, sensor/localization noise)
        k = len(env_ids)
        cfg = self.cfg
        dev = self.device
        self.delay_e[env_ids] = torch.randint(
            cfg.dr_delay_steps[0], cfg.dr_delay_steps[1] + 1, (k,), device=dev)
        self.vmax_e[env_ids] = cfg.v_max * (
            cfg.dr_speed_scale[0] + torch.rand(k, device=dev)
            * (cfg.dr_speed_scale[1] - cfg.dr_speed_scale[0]))
        self.amax_e[env_ids] = cfg.a_max * (
            cfg.dr_speed_scale[0] + torch.rand(k, device=dev)
            * (cfg.dr_speed_scale[1] - cfg.dr_speed_scale[0]))
        self.noise_mult_e[env_ids] = cfg.dr_noise_mult[0] + torch.rand(k, device=dev) \
            * (cfg.dr_noise_mult[1] - cfg.dr_noise_mult[0])
        self.bearing_noise_e[env_ids] = cfg.dr_goal_bearing_noise[0] \
            + torch.rand(k, device=dev) \
            * (cfg.dr_goal_bearing_noise[1] - cfg.dr_goal_bearing_noise[0])
        # estimator drift: per-env OU step size; start at the stationary
        # distribution (episodes begin mid-drift, like a real run)
        self.drift_e[env_ids] = cfg.dr_goal_drift[0] + torch.rand(k, device=dev) \
            * (cfg.dr_goal_drift[1] - cfg.dr_goal_drift[0])
        stat_std = self.drift_e[env_ids] / math.sqrt(2.0 * cfg.goal_drift_pull)
        self.bearing_bias[env_ids] = torch.randn(k, device=dev) * stat_std
        self.dist_bias[env_ids] = torch.randn(k, device=dev) * stat_std * 0.5
        # progress baseline for the fresh track
        self.progress[env_ids] = self._arc_progress()[env_ids]


def run_smoke(num_envs: int, smoke_steps: int, track_type: str = "corridor"):
    """Random-action smoke test; called by roboracer_isaaclab_task.py."""
    cfg = RoboracerEnvCfg()
    cfg.track_type = track_type
    cfg.scene.num_envs = num_envs
    env = RoboracerEnv(cfg)
    obs0, _ = env.reset()
    # sanity: final-goal obs at spawn — distance should be positive and finite,
    # bearing can point anywhere (the route turns; goal is often off-axis)
    g = obs0["policy"][:, -3:]  # [dist/10m, sin(bearing), cos(bearing)]
    print(f"[Phase3] goal-obs at spawn: dist={10 * g[:, 0].mean():.2f} m "
          f"(route lengths ~4-30 m), |sin^2+cos^2-1|="
          f"{(g[:, 1] ** 2 + g[:, 2] ** 2 - 1).abs().max():.1e} (expect ~0)")
    print(f"[Phase3] Running: {env.num_envs} envs, obs={cfg.observation_space}, "
          f"act={cfg.action_space}, steer_max={cfg.steer_max_deg} deg, "
          f"v_max={cfg.v_max} m/s")

    ray_min = float("inf")
    ray_mean_acc = 0.0
    for i in range(smoke_steps):
        # biased-forward random actions so cars actually drive
        steer = torch.rand(env.num_envs, 1, device=env.device) * 2 - 1
        thr = torch.rand(env.num_envs, 1, device=env.device) * 0.6 + 0.4
        obs, rew, terminated, truncated, info = env.step(torch.cat([steer, thr], dim=1))
        rays = obs["policy"][:, : cfg.num_rays] * cfg.lidar_max_range
        ray_min = min(ray_min, float(rays.min()))
        ray_mean_acc += float(rays.mean())

    print(f"[Phase3] {smoke_steps} steps OK | "
          f"crashes={env.total_crashes} goals={env.total_goals} | "
          f"lidar min={ray_min:.3f} m mean={ray_mean_acc / smoke_steps:.3f} m")
    if ray_mean_acc / smoke_steps >= cfg.lidar_max_range - 1e-3:
        print("[Phase3] WARNING: LiDAR never saw a wall — track generation broken?")
    else:
        print("[Phase3] LiDAR sees track walls — geometry pipeline verified")
    env.close()
