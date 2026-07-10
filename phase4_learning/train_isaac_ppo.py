#!/usr/bin/env python3
"""
Phase 4: train PPO+GRU on the Phase 3 Isaac Lab maze env (rsl-rl).

Replaces the old train_policy.py toy-env pipeline. Uses the design's algorithm
choice (SYSTEM_DESIGN.md section 3): recurrent PPO, 20 Hz control, obs = 64 LiDAR
+ dynamics + goal bearing/distance.

Run (server, conda env `isaaclab`):
  cd ~/roboracer_project/phase4_learning
  export OMNI_KIT_ACCEPT_EULA=YES
  python train_isaac_ppo.py --headless --num_envs 2048 --max_iterations 1500

Monitor:
  tensorboard --logdir ~/roboracer_project/phase4_learning/logs --port 6006
  (tunnel: ssh -L 6006:localhost:6006 <server>)

Resume / play:
  checkpoints land in logs/roboracer_stage3/<timestamp>/model_<iter>.pt
"""

import argparse
import os
import sys
from datetime import datetime
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Roboracer Phase 4 PPO+GRU training")
parser.add_argument("--num_envs", type=int, default=2048)
parser.add_argument("--max_iterations", type=int, default=1500)
parser.add_argument("--seed", type=int, default=42)
parser.add_argument("--run_name", type=str, default="")
parser.add_argument("--resume", type=str, default="", help="checkpoint .pt to resume from")
parser.add_argument(
    "--vehicle_params",
    type=str,
    default=str(Path.home() / "roboracer_project/phase1_sysid/config/vehicle_params.yaml"),
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

os.environ["ROBORACER_VEHICLE_PARAMS"] = args_cli.vehicle_params

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# ---------------------------------------------------------------------------
import torch  # noqa: E402

from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab.utils.dict import print_dict  # noqa: E402
from isaaclab_rl.rsl_rl import (  # noqa: E402
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticRecurrentCfg,
    RslRlPpoAlgorithmCfg,
    RslRlVecEnvWrapper,
)

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "phase3_sim" / "envs"))
from roboracer_env import RoboracerEnv, RoboracerEnvCfg  # noqa: E402


def build_agent_cfg() -> RslRlOnPolicyRunnerCfg:
    return RslRlOnPolicyRunnerCfg(
        seed=args_cli.seed,
        device="cuda:0",
        num_steps_per_env=24,          # rollout horizon per env per iteration
        max_iterations=args_cli.max_iterations,
        save_interval=100,
        experiment_name="roboracer_stage3",
        run_name=args_cli.run_name,
        empirical_normalization=True,  # running obs normalization
        policy=RslRlPpoActorCriticRecurrentCfg(
            init_noise_std=1.0,
            actor_hidden_dims=[256, 128],
            critic_hidden_dims=[256, 128],
            activation="elu",
            rnn_type="gru",            # design section 3: PPO + GRU
            rnn_hidden_dim=256,
            rnn_num_layers=1,
        ),
        algorithm=RslRlPpoAlgorithmCfg(
            value_loss_coef=1.0,
            use_clipped_value_loss=True,
            clip_param=0.2,
            entropy_coef=0.005,
            num_learning_epochs=5,
            num_mini_batches=4,
            learning_rate=5.0e-4,
            schedule="adaptive",
            gamma=0.99,
            lam=0.95,
            desired_kl=0.01,
            max_grad_norm=1.0,
        ),
    )


def main():
    env_cfg = RoboracerEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = args_cli.seed
    env = RoboracerEnv(env_cfg)
    env = RslRlVecEnvWrapper(env)

    agent_cfg = build_agent_cfg()
    log_dir = Path(__file__).resolve().parent / "logs" / agent_cfg.experiment_name
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    if agent_cfg.run_name:
        stamp += f"_{agent_cfg.run_name}"
    log_dir = log_dir / stamp
    log_dir.mkdir(parents=True, exist_ok=True)

    cfg_dict = agent_cfg.to_dict()
    print_dict(cfg_dict, nesting=1)

    runner = OnPolicyRunner(env, cfg_dict, log_dir=str(log_dir), device=agent_cfg.device)
    if args_cli.resume:
        runner.load(args_cli.resume)
        print(f"[Phase4] resumed from {args_cli.resume}")

    print(f"[Phase4] training: {args_cli.num_envs} envs x {agent_cfg.num_steps_per_env} steps "
          f"x {args_cli.max_iterations} iters "
          f"(~{args_cli.num_envs * agent_cfg.num_steps_per_env * args_cli.max_iterations / 1e6:.0f}M env steps)")
    print(f"[Phase4] logs: {log_dir}")

    runner.learn(num_learning_iterations=args_cli.max_iterations, init_at_random_ep_len=True)

    print("[Phase4] training complete")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
