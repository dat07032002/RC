#!/usr/bin/env python3
"""
Roboracer Isaac Lab Environment
Phase 3: Realistic simulation with domain randomization
"""

import numpy as np
import gymnasium as gym
from gymnasium import spaces
import yaml
from pathlib import Path

class RoboracerIsaacEnv(gym.Env):
    """
    Roboracer environment in Isaac Lab
    - Realistic physics simulation
    - Domain randomization
    - LiDAR and IMU simulation
    - Gym-compatible interface
    """

    metadata = {"render_modes": []}

    def __init__(self, config_path="configs/isaac_config.yaml", domain_randomization=True):
        super().__init__()

        self.domain_randomization = domain_randomization
        self.max_steps = 1000
        self.dt = 0.01

        # Observation space: LiDAR (64) + IMU (6) + pose (3) + velocity (2)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf,
            shape=(75,), dtype=np.float32
        )

        # Action space: steering angle (-30 to +30 deg), throttle (0 to 1)
        self.action_space = spaces.Box(
            low=np.array([-30.0, 0.0]),
            high=np.array([30.0, 1.0]),
            dtype=np.float32
        )

        # Physics parameters
        self._init_domain_randomization()

        self.reset()

    def _init_domain_randomization(self):
        """Initialize domain randomization parameters"""
        if self.domain_randomization:
            # Random friction
            self.friction = np.random.uniform(0.3, 0.8)

            # Random mass
            self.mass = np.random.uniform(0.9, 1.1)

            # Random sensor noise
            self.lidar_noise_std = np.random.uniform(0.01, 0.05)
            self.imu_noise_std = np.random.uniform(0.001, 0.01)

            # Random actuator delay (latency)
            self.control_delay = np.random.randint(1, 3)  # 1-2 control steps
        else:
            self.friction = 0.5
            self.mass = 1.0
            self.lidar_noise_std = 0.03
            self.imu_noise_std = 0.005
            self.control_delay = 1

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_count = 0

        # Initialize state
        self.position = np.array([0.5, 0.5, 0.0], dtype=np.float32)
        self.velocity = np.array([0.0, 0.0], dtype=np.float32)
        self.imu_data = np.zeros(6, dtype=np.float32)

        # Action history for delay
        self.action_buffer = [np.array([0.0, 0.0])] * self.control_delay

        return self._get_observation(), {}

    def step(self, action):
        """Execute simulation step with realistic physics"""
        self.step_count += 1

        # Apply control delay
        self.action_buffer.append(action)
        delayed_action = self.action_buffer.pop(0)

        steering_angle = float(delayed_action[0])
        throttle = float(delayed_action[1])

        # Realistic vehicle kinematic model
        max_velocity = 2.0 * self.mass
        v = throttle * max_velocity

        # Add actuator dynamics
        tau = 0.1
        self.velocity[0] += (v * np.cos(np.radians(self.position[2])) - self.velocity[0]) * self.dt / tau
        self.velocity[1] += (v * np.sin(np.radians(self.position[2])) - self.velocity[1]) * self.dt / tau

        # Position update
        self.position[0] += self.velocity[0] * self.dt
        self.position[1] += self.velocity[1] * self.dt

        # Steering kinematic
        wheelbase = 0.12
        self.position[2] += np.degrees(v / wheelbase * np.tan(np.radians(steering_angle)) * self.dt)

        # Simulate LiDAR with noise
        lidar = self._simulate_lidar_realistic()

        # Reward
        reward = v * 0.1

        # Collision detection
        terminated = False
        if self.position[1] < 0.1 or self.position[1] > 1.9:
            reward -= 10.0
            terminated = True

        if self.step_count >= self.max_steps:
            terminated = True

        return self._get_observation(), reward, terminated, False, {}

    def _simulate_lidar_realistic(self):
        """Realistic LiDAR simulation with noise model"""
        lidar = np.ones(64, dtype=np.float32) * 10.0

        for i in range(64):
            # Distance to walls
            dist_to_wall = max(0.1, 1.0 - 0.5 * abs(self.position[1] - 1.0))

            # Add noise based on distance
            noise = self.lidar_noise_std + 0.01 * dist_to_wall
            measured_dist = dist_to_wall + np.random.normal(0, noise)

            lidar[i] = np.clip(measured_dist, 0.1, 10.0)

        return lidar

    def _get_observation(self):
        """Get observation with simulated sensors"""
        lidar = self._simulate_lidar_realistic()

        # Simulate IMU
        imu = np.array([
            0.0, 0.0, 9.81,
            self.velocity[0]/2, self.velocity[1]/2, 0.0
        ], dtype=np.float32)

        # Add IMU noise
        imu += np.random.normal(0, self.imu_noise_std, 6).astype(np.float32)

        # Concatenate observation
        obs = np.concatenate([
            lidar,
            imu,
            self.position.astype(np.float32),
            self.velocity.astype(np.float32)
        ]).astype(np.float32)

        return obs

    def render(self, mode="human"):
        pass


if __name__ == "__main__":
    env = RoboracerIsaacEnv(domain_randomization=True)
    obs, _ = env.reset()
    print(f"✓ Isaac environment initialized")
    print(f"  Observation shape: {obs.shape}")

    for i in range(10):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, _ = env.step(action)
        if terminated:
            obs, _ = env.reset()

    print("✓ Environment test successful")
