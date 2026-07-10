#!/usr/bin/env python3
"""
Physics-car validation: does the USD articulation behave like the measured car?

Test A (straight): full throttle 4 s -> expect v -> ~v_max (0.57 m/s) and
  a roughly a_max-limited ramp (0.61 m/s^2).
Test B (full left circle): constant speed, full steering -> expect yaw rate
  ~ v * tan(steer) / wheelbase (bicycle prediction; tire slip makes it lower).

Run (server):
  export OMNI_KIT_ACCEPT_EULA=YES
  python roboracer_phys_smoke.py --headless
"""

import argparse
import math
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--num_envs", type=int, default=4)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent / "envs"))
from roboracer_phys_env import RoboracerPhysEnv, RoboracerPhysEnvCfg  # noqa: E402


class OpenGroundEnv(RoboracerPhysEnv):
    """Terminations disabled: dynamics validation must not be interrupted by
    crash-resets (a full-lock circle inside a maze corridor hits walls, and
    averaging yaw across reset teleports produces garbage understeer)."""

    def _get_dones(self):
        self._sync_car_state()
        never = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self._min_wall_dist = torch.full((self.num_envs,), 1e9, device=self.device)
        self.crashed = never
        self.reached_goal = never
        return never, never


def main():
    cfg = RoboracerPhysEnvCfg()
    cfg.scene.num_envs = args_cli.num_envs
    env = OpenGroundEnv(cfg)
    env.reset()
    print(f"[PhysSmoke] joints: {env.robot.joint_names}")

    dev = env.device
    n = env.num_envs

    # --- Test A: straight, full throttle, 4 s (80 control steps) ----------
    v_log = []
    for i in range(80):
        act = torch.zeros(n, 2, device=dev)
        act[:, 1] = 1.0  # full throttle
        env.step(act)
        v_log.append(float(env.car_state[:, 3].mean()))
    v_end = v_log[-1]
    # accel from the first second of the ramp
    a_meas = (v_log[19] - v_log[0]) / 1.0
    print(f"[PhysSmoke] A straight: v_end={v_end:.3f} m/s (target {cfg.v_max}), "
          f"ramp accel~{a_meas:.3f} m/s^2 (cap {cfg.a_max})")

    # --- Test B: full left at steady speed, 6 s ---------------------------
    yaw0 = env.car_state[:, 2].clone()
    yaw_prev = yaw0.clone()
    yaw_unwrapped = torch.zeros(n, device=dev)
    v_sum, cnt = 0.0, 0
    for i in range(120):
        act = torch.zeros(n, 2, device=dev)
        act[:, 0] = 1.0  # full left
        act[:, 1] = 0.6
        env.step(act)
        yaw = env.car_state[:, 2]
        dy = yaw - yaw_prev
        dy = torch.atan2(torch.sin(dy), torch.cos(dy))  # wrap
        yaw_unwrapped += dy
        yaw_prev = yaw
        if i >= 40:  # steady-state only
            v_sum += float(env.car_state[:, 3].mean())
            cnt += 1
    v_ss = v_sum / cnt
    # yaw rate over the steady window
    yr_meas = float(yaw_unwrapped.mean()) / 6.0
    steer = float(env.car_state[:, 4].mean())
    yr_pred = v_ss * math.tan(steer) / cfg.wheelbase if abs(steer) > 1e-3 else float("nan")
    radius = v_ss / abs(yr_meas) if abs(yr_meas) > 1e-3 else float("inf")
    print(f"[PhysSmoke] B circle: v={v_ss:.3f} m/s, steer={math.degrees(steer):.1f} deg, "
          f"yaw_rate={yr_meas:.3f} rad/s (bicycle predicts {yr_pred:.3f}), "
          f"turn radius={radius:.2f} m")

    # turning passes if within 70-110% of the bicycle prediction (tire slip
    # makes the real/physical value a bit lower, never higher)
    ok_v = 0.35 <= v_end <= 0.75
    ok_yr = 0.70 * abs(yr_pred) <= abs(yr_meas) <= 1.10 * abs(yr_pred)
    print(f"[PhysSmoke] verdict: speed {'OK' if ok_v else 'FAIL'}, "
          f"turning {'OK' if ok_yr else 'FAIL'}")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
