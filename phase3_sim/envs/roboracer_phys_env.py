#!/usr/bin/env python3
"""
Phase 3 physics env: the maze task with a real PhysX Ackermann articulation
instead of the analytic bicycle model.

Subclasses RoboracerEnv, so track generation, LiDAR, goal-conditioning,
rewards and terminations are inherited unchanged. What changes:
  * the car is a USD articulation (build with assets/build_car_usd.py):
    steering = position-driven revolute joints (±25 deg, servo-lag filtered),
    wheels   = velocity-driven revolute joints (AWD), tires get PhysX friction
  * car_state [x, y, yaw, v, steer] is READ BACK from the simulation each step
    instead of integrated — tire slip, inertia and drive limits now exist.

Import only after AppLauncher is up. Set ROBORACER_CAR_USD to override the
asset path.
"""

import math
import os
from pathlib import Path

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import euler_xyz_from_quat

from roboracer_env import RoboracerEnv, RoboracerEnvCfg, VEHICLE

CAR_USD = os.environ.get(
    "ROBORACER_CAR_USD",
    str(Path.home() / "roboracer_project/phase3_sim/assets/roboracer_car.usd"),
)

WHEEL_RADIUS = 0.045  # measured 2026-07-11; keep in sync with build_car_usd.py

ROBORACER_CAR_CFG = ArticulationCfg(
    prim_path="/World/envs/env_.*/Robot",
    spawn=sim_utils.UsdFileCfg(usd_path=CAR_USD),
    init_state=ArticulationCfg.InitialStateCfg(pos=(0.15, 0.0, 0.05)),
    actuators={
        # servo: stiff position drive; command-side first-order filter models tau
        "steer": ImplicitActuatorCfg(
            joint_names_expr=["steer_.*"],
            stiffness=40.0, damping=2.0, effort_limit_sim=10.0,
        ),
        # front wheels: free-rolling. Driving them at a forced common speed
        # saturates tire friction with longitudinal slip in turns (locked-diff
        # understeer); free fronts approximate the real diff far better.
        "wheels_front": ImplicitActuatorCfg(
            joint_names_expr=["wheel_F.*"],
            stiffness=0.0, damping=0.0, effort_limit_sim=0.0,
        ),
        # rear wheels: soft velocity drive with a small torque cap (~open diff).
        # accel needs only ~0.05 Nm/wheel; the cap stops slip-forcing in turns.
        "wheels_rear": ImplicitActuatorCfg(
            joint_names_expr=["wheel_R.*"],
            stiffness=0.0, damping=0.3, effort_limit_sim=0.2,
        ),
    },
)


@configclass
class RoboracerPhysEnvCfg(RoboracerEnvCfg):
    robot: ArticulationCfg = ROBORACER_CAR_CFG
    track_width_m: float = VEHICLE["track_width"]
    # physical cars need real spacing (analytic walls aren't physical, but
    # cross-env car-car contact is filtered anyway; spacing keeps spawns clean)
    def __post_init__(self):
        self.scene.env_spacing = 6.0


class RoboracerPhysEnv(RoboracerEnv):
    cfg: RoboracerPhysEnvCfg

    def __init__(self, cfg: RoboracerPhysEnvCfg, render_mode=None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        self._steer_ids, _ = self.robot.find_joints(["steer_FL", "steer_FR"])
        # order matters for the Ackermann speed split: [RL (left), RR (right)]
        self._wheel_ids, _ = self.robot.find_joints(["wheel_RL", "wheel_RR"], preserve_order=True)
        n = self.num_envs
        self._steer_state = torch.zeros(n, device=self.device)   # servo-lag filter
        self._v_target = torch.zeros(n, device=self.device)      # accel slew

    # ---- scene: spawn the articulation ------------------------------------
    def _setup_scene(self):
        self.robot = Articulation(self.cfg.robot)
        ground = sim_utils.GroundPlaneCfg(
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.0, dynamic_friction=0.9,  # [P] floor friction
            )
        )
        ground.func("/World/ground", ground)
        light = sim_utils.DomeLightCfg(intensity=2000.0)
        light.func("/World/Light", light)
        self.scene.clone_environments(copy_from_source=False)
        self.scene.filter_collisions(global_prim_paths=["/World/ground"])
        self.scene.articulations["robot"] = self.robot

    # ---- actions: joint targets instead of state integration --------------
    def _apply_action(self):
        dt = self.cfg.sim.dt
        steer_cmd = self._act[:, 0] * self.steer_max_rad
        throttle = (self._act[:, 1] + 1.0) / 2.0

        # servo lag (measured tau) applied to the position target
        self._steer_state = self._steer_state + (
            steer_cmd - self._steer_state) * dt / self.cfg.steer_tau_s
        # accel-limited speed target (measured a_max), like the real slew limiter
        v_cmd = throttle * self.cfg.v_max
        dv = (v_cmd - self._v_target).clamp(-self.cfg.a_max * dt, self.cfg.a_max * dt)
        self._v_target = self._v_target + dv

        steer_t = self._steer_state.unsqueeze(1).repeat(1, len(self._steer_ids))
        self.robot.set_joint_position_target(steer_t, joint_ids=self._steer_ids)
        # Ackermann rear-speed split (differential): inner wheel slower.
        # Left turn (steer>0): left wheel (RL) is inner.
        w = self._v_target / WHEEL_RADIUS
        ratio = self.cfg.track_width_m * torch.tan(self._steer_state) / (2.0 * self.cfg.wheelbase)
        omega_t = torch.stack([w * (1.0 - ratio), w * (1.0 + ratio)], dim=1)  # [RL, RR]
        self.robot.set_joint_velocity_target(omega_t, joint_ids=self._wheel_ids)

    # ---- read car_state back from physics ---------------------------------
    def _sync_car_state(self):
        pos = self.robot.data.root_pos_w[:, :2] - self.scene.env_origins[:, :2]
        _, _, yaw = euler_xyz_from_quat(self.robot.data.root_quat_w)
        v = self.robot.data.root_lin_vel_b[:, 0]
        steer = self.robot.data.joint_pos[:, self._steer_ids].mean(dim=1)
        self.car_state = torch.stack([pos[:, 0], pos[:, 1], yaw, v, steer], dim=1)

    def _get_dones(self):
        self._sync_car_state()
        return super()._get_dones()

    # ---- reset: place the articulation at the local spawn -----------------
    def _reset_idx(self, env_ids):
        super()._reset_idx(env_ids)  # regenerates track, sets car_state spawn
        k = len(env_ids)
        spawn = self.car_state[env_ids]
        pos = torch.zeros(k, 3, device=self.device)
        pos[:, :2] = spawn[:, :2] + self.scene.env_origins[env_ids, :2]
        pos[:, 2] = 0.05
        half_yaw = spawn[:, 2] / 2.0
        quat = torch.zeros(k, 4, device=self.device)
        quat[:, 0] = torch.cos(half_yaw)
        quat[:, 3] = torch.sin(half_yaw)
        self.robot.write_root_pose_to_sim(torch.cat([pos, quat], dim=1), env_ids=env_ids)
        self.robot.write_root_velocity_to_sim(
            torch.zeros(k, 6, device=self.device), env_ids=env_ids)
        zeros = torch.zeros(k, self.robot.num_joints, device=self.device)
        self.robot.write_joint_state_to_sim(zeros, zeros, env_ids=env_ids)
        self._steer_state[env_ids] = 0.0
        self._v_target[env_ids] = 0.0
