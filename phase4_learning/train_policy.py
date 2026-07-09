#!/usr/bin/env python3
"""
Phase 4: Policy Learning
Train PPO control policy on simulated roboracer environment
"""

import os
import json
import argparse
from pathlib import Path
from datetime import datetime

import numpy as np
import torch
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor


class RoboracerEnv(gym.Env):
    """Simple roboracer maze navigation environment"""

    metadata = {"render_modes": []}

    def __init__(self, config=None):
        super().__init__()
        self.config = config or {}
        self.max_steps = self.config.get("max_steps", 1000)

        # Observation: LiDAR (64) + IMU (6) + pose (3) + velocity (2) = 75 dims
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf,
            shape=(75,), dtype=np.float32
        )

        # Action: steering angle (-30 to +30 deg), throttle (0 to 1)
        self.action_space = gym.spaces.Box(
            low=np.array([-30.0, 0.0]),
            high=np.array([30.0, 1.0]),
            dtype=np.float32
        )

        self.reset()

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_count = 0
        self.position = np.array([0.5, 0.5, 0.0], dtype=np.float32)
        self.velocity = np.array([0.0, 0.0], dtype=np.float32)
        return self._get_observation(), {}

    def step(self, action):
        self.step_count += 1

        steering_angle = float(action[0])
        throttle = float(action[1])

        # Simple kinematic model
        dt = 0.01
        v = throttle * 2.0

        self.velocity[0] = v * np.cos(np.radians(self.position[2]))
        self.velocity[1] = v * np.sin(np.radians(self.position[2]))

        self.position[0] += self.velocity[0] * dt
        self.position[1] += self.velocity[1] * dt
        self.position[2] += np.degrees(v / 0.1 * np.tan(np.radians(steering_angle)) * dt)

        # Reward: forward movement
        reward = v * 0.1

        # Collision penalty
        terminated = False
        if self.position[1] < 0.1 or self.position[1] > 1.9:
            reward -= 10.0
            terminated = True

        if self.step_count >= self.max_steps:
            terminated = True

        return self._get_observation(), reward, terminated, False, {}

    def _get_observation(self):
        # Simulated LiDAR
        lidar = np.ones(64, dtype=np.float32) * 5.0
        imu = np.zeros(6, dtype=np.float32)

        obs = np.concatenate([
            lidar,
            imu,
            self.position.astype(np.float32),
            self.velocity.astype(np.float32)
        ])
        return obs


def main(args):
    log_dir = Path(args.logdir)
    log_dir.mkdir(parents=True, exist_ok=True)
    checkpoint_dir = log_dir / "checkpoints"
    checkpoint_dir.mkdir(exist_ok=True)

    print(f"\n🚀 Phase 4: Policy Learning")
    print(f"   Logdir: {log_dir}")
    print(f"   GPU: {torch.cuda.get_device_name(args.gpu) if torch.cuda.is_available() else 'CPU'}")
    print(f"   Timestamp: {datetime.now().isoformat()}\n")

    # Create environment
    print("Creating environment...")
    env = RoboracerEnv(config={"max_steps": args.max_steps})
    env = Monitor(env, str(log_dir / "monitor.csv"))

    # Train PPO
    print(f"Training PPO for {args.steps} steps...")

    device = f"cuda:{args.gpu}" if torch.cuda.is_available() else "cpu"

    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=args.lr,
        n_steps=args.n_steps,
        batch_size=args.batch_size,
        n_epochs=args.n_epochs,
        gamma=args.gamma,
        gae_lambda=args.gae_lambda,
        verbose=1,
        tensorboard_log=str(log_dir / "tensorboard"),
        device=device
    )

    # Checkpoint callback
    checkpoint_callback = CheckpointCallback(
        save_freq=max(args.steps // 10, 1000),
        save_path=str(checkpoint_dir),
        name_prefix="ppo"
    )

    # Train
    model.learn(
        total_timesteps=args.steps,
        callback=checkpoint_callback,
        log_interval=10
    )

    # Save final model
    final_path = log_dir / "ppo_final.zip"
    model.save(final_path)
    print(f"\n✓ Model saved: {final_path}")

    # Save metadata
    metadata = {
        "algorithm": "PPO",
        "total_steps": args.steps,
        "learning_rate": args.lr,
        "batch_size": args.batch_size,
        "timestamp": datetime.now().isoformat(),
        "device": device
    }

    with open(log_dir / "metadata.json", "w") as f:
        json.dump(metadata, f, indent=2)

    print(f"✓ Training complete!")
    print(f"\nResults saved:")
    print(f"  Model: {final_path}")
    print(f"  Logs: {log_dir}/tensorboard")
    print(f"\nView training with:")
    print(f"  tensorboard --logdir {log_dir}/tensorboard")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train roboracer policy")
    parser.add_argument("--logdir", default="logs/ppo_v1", help="Logging directory")
    parser.add_argument("--steps", type=int, default=100000, help="Training steps")
    parser.add_argument("--lr", type=float, default=3e-4, help="Learning rate")
    parser.add_argument("--batch-size", type=int, default=64, help="Batch size")
    parser.add_argument("--n-steps", type=int, default=2048, help="Steps per epoch")
    parser.add_argument("--n-epochs", type=int, default=10, help="Number of epochs")
    parser.add_argument("--gamma", type=float, default=0.99, help="Discount factor")
    parser.add_argument("--gae-lambda", type=float, default=0.95, help="GAE lambda")
    parser.add_argument("--max-steps", type=int, default=1000, help="Max episode steps")
    parser.add_argument("--gpu", type=int, default=0, help="GPU device ID")

    args = parser.parse_args()
    main(args)
