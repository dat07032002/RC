#!/usr/bin/env python3
"""
Curriculum gate evaluation: run a trained checkpoint deterministically and
report success / crash / timeout rates over completed episodes.

Run (server):
  export OMNI_KIT_ACCEPT_EULA=YES
  python eval_policy.py --headless --track_type room --obstacles 0,0 \
      --checkpoint logs/roboracer_stage3/<run>/model_399.pt
"""

import argparse
import os
import sys
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser()
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--num_envs", type=int, default=512)
parser.add_argument("--steps", type=int, default=1500, help="eval horizon (control steps)")
parser.add_argument("--track_type", type=str, default="room", choices=["corridor", "room"])
parser.add_argument("--obstacles", type=str, default="")
parser.add_argument("--seed", type=int, default=1234, help="held-out eval seed")
parser.add_argument(
    "--vehicle_params", type=str,
    default=str(Path.home() / "roboracer_project/phase1_sysid/config/vehicle_params.yaml"))
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
os.environ["ROBORACER_VEHICLE_PARAMS"] = args_cli.vehicle_params

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402
from isaaclab_rl.rsl_rl import (  # noqa: E402
    RslRlOnPolicyRunnerCfg,
    RslRlPpoActorCriticRecurrentCfg,
    RslRlPpoAlgorithmCfg,
    RslRlVecEnvWrapper,
)

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "phase3_sim" / "envs"))
from roboracer_env import RoboracerEnv, RoboracerEnvCfg  # noqa: E402


def build_agent_cfg() -> RslRlOnPolicyRunnerCfg:
    """Must match train_isaac_ppo.py exactly (importing it would run its CLI)."""
    return RslRlOnPolicyRunnerCfg(
        seed=args_cli.seed,
        device="cuda:0",
        num_steps_per_env=24,
        max_iterations=1,
        save_interval=100,
        experiment_name="roboracer_stage3_eval",
        empirical_normalization=True,
        policy=RslRlPpoActorCriticRecurrentCfg(
            init_noise_std=1.0,
            actor_hidden_dims=[256, 128],
            critic_hidden_dims=[256, 128],
            activation="elu",
            rnn_type="gru",
            rnn_hidden_dim=256,
            rnn_num_layers=1,
        ),
        algorithm=RslRlPpoAlgorithmCfg(
            value_loss_coef=1.0, use_clipped_value_loss=True, clip_param=0.2,
            entropy_coef=0.005, num_learning_epochs=5, num_mini_batches=4,
            learning_rate=5.0e-4, schedule="adaptive", gamma=0.99, lam=0.95,
            desired_kl=0.01, max_grad_norm=1.0,
        ),
    )


def main():
    cfg = RoboracerEnvCfg()
    cfg.track_type = args_cli.track_type
    if args_cli.obstacles:
        lo, hi = (int(x) for x in args_cli.obstacles.split(","))
        if args_cli.track_type == "room":
            cfg.dr_room_obstacles = (lo, hi)
        else:
            cfg.dr_num_obstacles = (lo, hi)
    cfg.scene.num_envs = args_cli.num_envs
    cfg.seed = args_cli.seed

    env = RoboracerEnv(cfg)
    env.rng = __import__("numpy").random.default_rng(args_cli.seed)  # held-out worlds
    wrapped = RslRlVecEnvWrapper(env)

    agent_cfg = build_agent_cfg()
    runner = OnPolicyRunner(wrapped, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    runner.load(args_cli.checkpoint)
    policy = runner.get_inference_policy(device=agent_cfg.device)
    print(f"[Eval] loaded {args_cli.checkpoint}")

    obs, _ = wrapped.get_observations()
    env.total_goals = 0
    env.total_crashes = 0
    episodes = 0
    ep_len_acc = 0
    for i in range(args_cli.steps):
        with torch.no_grad():
            actions = policy(obs)
        obs, _, dones, _ = wrapped.step(actions)
        n_done = int(dones.sum())
        if n_done:
            episodes += n_done
            # episode_length_buf was reset for done envs; approximate via dones
            net = getattr(runner.alg, "policy", None) or getattr(runner.alg, "actor_critic")
            net.reset(dones.to(torch.bool))
        ep_len_acc += args_cli.num_envs

    goals, crashes = env.total_goals, env.total_crashes
    timeouts = max(episodes - goals - crashes, 0)
    print(f"[Eval] {episodes} episodes over {args_cli.steps} steps x {args_cli.num_envs} envs")
    if episodes:
        print(f"[Eval] success: {goals} ({100 * goals / episodes:.1f}%) | "
              f"crash: {crashes} ({100 * crashes / episodes:.1f}%) | "
              f"timeout: {timeouts} ({100 * timeouts / episodes:.1f}%)")
        print(f"[Eval] mean steps/episode: {ep_len_acc / episodes:.0f}")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
