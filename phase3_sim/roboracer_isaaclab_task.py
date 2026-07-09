#!/usr/bin/env python3
"""
Phase 3: Roboracer maze-navigation task for Isaac Lab (DirectRLEnv).

STATUS: SCAFFOLD — runs, but three inputs must come from earlier phases:
  1. vehicle_params.yaml   <- Phase 1 measurements (wheelbase, steering gain/lag,
                              throttle curve, LiDAR noise a+b model, IMU bias)
  2. Maze layout           <- Phase 2 occupancy map (or randomized per DR plan)
  3. Car USD asset         <- simple Ackermann articulation (TODO Week 4)

Run (on training server, conda env `isaaclab`):
  cd ~/roboracer_project/isaac_lab_env/IsaacLab_official
  ./isaaclab.sh -p ~/roboracer_project/phase3_sim/roboracer_isaaclab_task.py --headless

Domain randomization targets (from comprehensive_plan.md):
  corridor width 0.5-2.0 m | turns 3-8 | obstacle density 0-50%
  LiDAR noise +-5 cm | IMU gyro bias +-50 deg/s | steering delay 20-100 ms
  trajectory mix: 68% randomized mazes / 25% real track / 7% edge cases
"""

import argparse
from pathlib import Path

import yaml

from isaaclab.app import AppLauncher

# ---------------------------------------------------------------------------
# CLI + app launch (must happen before other isaaclab imports)
# ---------------------------------------------------------------------------
parser = argparse.ArgumentParser(description="Roboracer Phase 3 Isaac Lab task")
parser.add_argument("--num_envs", type=int, default=64, help="parallel environments")
parser.add_argument(
    "--vehicle_params",
    type=str,
    default=str(Path.home() / "roboracer_project/phase1_sysid/config/vehicle_params.yaml"),
    help="Phase 1 measured parameters (YAML)",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ---------------------------------------------------------------------------
# Everything below requires the app to be running
# ---------------------------------------------------------------------------
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.scene import InteractiveSceneCfg  # noqa: E402
from isaaclab.sim import SimulationCfg  # noqa: E402
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402


def load_vehicle_params(path: str) -> dict:
    """Load Phase 1 measurements; fall back to plan defaults with a loud warning."""
    p = Path(path)
    if p.exists():
        with open(p) as f:
            params = yaml.safe_load(f)
        print(f"[Phase3] Loaded measured vehicle params: {p}")
        return params
    print("=" * 70)
    print("[Phase3] WARNING: vehicle_params.yaml not found — using PLACEHOLDER")
    print("         dynamics. Results are NOT valid for sim-to-real transfer")
    print("         until Phase 1 measurements are synced to the server.")
    print("=" * 70)
    return {
        "kinematics": {"wheelbase": 0.32, "track_width": 0.20},  # typical 1/10 RC
        "steering": {"angle_max_deg": 30.0, "response_tau_s": 0.08},
        "throttle": {"max_velocity": 2.0, "max_acceleration": 1.0},
        "sensors": {
            "lidar": {"noise_a": 0.03, "noise_b": 0.01, "num_rays": 36, "max_range": 10.0},
            "imu": {"gyro_noise": 0.02, "accel_noise": 0.05},
        },
        "latency": {"total_ms": 60.0},
    }


VEHICLE = load_vehicle_params(args_cli.vehicle_params)


@configclass
class RoboracerEnvCfg(DirectRLEnvCfg):
    """Config for the maze-navigation task. Obs/action match the plan:
    obs = LiDAR rays + velocity + steering state; act = [steer, throttle]."""

    # timing: 100 Hz physics, 20 Hz control (matches Jetson deployment rate)
    decimation = 5
    episode_length_s = 30.0

    num_rays: int = VEHICLE["sensors"]["lidar"]["num_rays"]
    action_space = 2                      # [steering, throttle]
    observation_space = num_rays + 4      # rays + v + yaw_rate + prev actions
    state_space = 0

    sim: SimulationCfg = SimulationCfg(dt=0.01, render_interval=5)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=64, env_spacing=6.0, replicate_physics=True
    )

    # --- measured dynamics (Phase 1) ---
    wheelbase: float = VEHICLE["kinematics"]["wheelbase"]
    steer_max_deg: float = VEHICLE["steering"]["angle_max_deg"]
    steer_tau_s: float = VEHICLE["steering"]["response_tau_s"]
    v_max: float = VEHICLE["throttle"]["max_velocity"]
    a_max: float = VEHICLE["throttle"]["max_acceleration"]

    # --- sensor noise model (Phase 1): std = a + b * range ---
    lidar_noise_a: float = VEHICLE["sensors"]["lidar"]["noise_a"]
    lidar_noise_b: float = VEHICLE["sensors"]["lidar"]["noise_b"]
    lidar_max_range: float = VEHICLE["sensors"]["lidar"]["max_range"]

    # --- domain randomization ranges (comprehensive_plan.md) ---
    dr_corridor_width = (0.5, 2.0)
    dr_num_turns = (3, 8)
    dr_obstacle_density = (0.0, 0.5)
    dr_action_delay_ms = (20.0, 100.0)


class RoboracerEnv(DirectRLEnv):
    """Maze navigation with bicycle-model car and simulated planar LiDAR.

    Week 4 TODO list (in order):
      [ ] _setup_scene: spawn maze walls from occupancy grid / DR generator
      [ ] _setup_scene: spawn car articulation (USD) or kinematic proxy
      [ ] LiDAR: replace ray-cast stub with isaaclab RayCaster sensor
      [ ] rewards: goal_progress - collision_penalty - smoothness_cost
      [ ] action delay buffer sized from measured latency (Phase 1)
    """

    cfg: RoboracerEnvCfg

    def __init__(self, cfg: RoboracerEnvCfg, render_mode=None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n = self.num_envs
        dev = self.device
        # bicycle-model state: [x, y, yaw, v, steer_angle]
        self.car_state = torch.zeros(n, 5, device=dev)
        self.prev_actions = torch.zeros(n, 2, device=dev)
        # action delay buffer (steps at 20 Hz control) from measured latency
        delay_steps = max(1, round(VEHICLE["latency"]["total_ms"] / 50.0))
        self.action_buf = torch.zeros(delay_steps, n, 2, device=dev)

    # ---- scene ----------------------------------------------------------
    def _setup_scene(self):
        # ground plane + light; maze walls are Week 4 work
        self.cfg.scene.ground = sim_utils.GroundPlaneCfg()
        spawn_light = sim_utils.DomeLightCfg(intensity=2000.0)
        spawn_light.func("/World/Light", spawn_light)

    # ---- MDP ------------------------------------------------------------
    def _pre_physics_step(self, actions: torch.Tensor):
        # push into delay buffer, pop oldest (models measured latency)
        self.action_buf = torch.roll(self.action_buf, shifts=-1, dims=0)
        self.action_buf[-1] = actions.clamp(-1.0, 1.0)
        self._act = self.action_buf[0]

    def _apply_action(self):
        steer_cmd = self._act[:, 0] * torch.deg2rad(
            torch.tensor(self.cfg.steer_max_deg, device=self.device)
        )
        throttle = (self._act[:, 1] + 1.0) / 2.0  # [-1,1] -> [0,1]

        dt = self.cfg.sim.dt
        x, y, yaw, v, steer = self.car_state.unbind(dim=1)
        # first-order steering lag (measured tau)
        steer = steer + (steer_cmd - steer) * dt / self.cfg.steer_tau_s
        # longitudinal: accel-limited approach to commanded speed
        v_cmd = throttle * self.cfg.v_max
        dv = (v_cmd - v).clamp(-self.cfg.a_max * dt, self.cfg.a_max * dt)
        v = v + dv
        # bicycle kinematics with measured wheelbase
        yaw = yaw + v / self.cfg.wheelbase * torch.tan(steer) * dt
        x = x + v * torch.cos(yaw) * dt
        y = y + v * torch.sin(yaw) * dt
        self.car_state = torch.stack([x, y, yaw, v, steer], dim=1)

    def _get_observations(self) -> dict:
        rays = self._raycast_lidar()
        # measured noise model: std = a + b * range
        noise_std = self.cfg.lidar_noise_a + self.cfg.lidar_noise_b * rays
        rays = rays + torch.randn_like(rays) * noise_std
        obs = torch.cat(
            [
                rays / self.cfg.lidar_max_range,
                self.car_state[:, 3:4] / self.cfg.v_max,   # v
                self.car_state[:, 4:5],                     # steer
                self.prev_actions,
            ],
            dim=1,
        )
        self.prev_actions = self._act.clone()
        return {"policy": obs}

    def _raycast_lidar(self) -> torch.Tensor:
        # WEEK-4 STUB: returns max range until maze walls + RayCaster exist.
        return torch.full(
            (self.num_envs, self.cfg.num_rays),
            self.cfg.lidar_max_range,
            device=self.device,
        )

    def _get_rewards(self) -> torch.Tensor:
        # plan: reward = goal_progress - collision_penalty - smoothness_cost
        progress = self.car_state[:, 3] * self.cfg.sim.dt * self.cfg.decimation
        smoothness = (self._act - self.prev_actions).square().sum(dim=1)
        return progress - 0.05 * smoothness

    def _get_dones(self):
        time_out = self.episode_length_buf >= self.max_episode_length - 1
        crashed = torch.zeros_like(time_out)  # needs walls (Week 4)
        return crashed, time_out

    def _reset_idx(self, env_ids):
        super()._reset_idx(env_ids)
        self.car_state[env_ids] = 0.0
        self.action_buf[:, env_ids] = 0.0
        self.prev_actions[env_ids] = 0.0


def main():
    cfg = RoboracerEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    env = RoboracerEnv(cfg)
    env.reset()
    print(f"[Phase3] Scaffold running: {env.num_envs} envs, "
          f"obs={cfg.observation_space}, act={cfg.action_space}")
    for i in range(200):
        actions = torch.rand(env.num_envs, 2, device=env.device) * 2.0 - 1.0
        obs, rew, terminated, truncated, info = env.step(actions)
    print("[Phase3] 200 steps OK — scaffold verified")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
