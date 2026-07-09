#!/usr/bin/env python3
import sys
sys.path.insert(0, '../envs')
from roboracer_isaac_env import RoboracerIsaacEnv
import numpy as np
from pathlib import Path

print("🎮 Phase 3: Collecting simulation dataset...")
env = RoboracerIsaacEnv(domain_randomization=True)

episodes = 0
total_steps = 0

while total_steps < 10000:
    obs, _ = env.reset()
    
    for step in range(env.max_steps):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, _ = env.step(action)
        total_steps += 1
        
        if terminated or truncated:
            break
    
    episodes += 1
    if episodes % 10 == 0:
        print(f"  Episodes: {episodes}, Steps: {total_steps}")

print(f"✓ Dataset collection complete!")
print(f"  Episodes: {episodes}, Total steps: {total_steps}")

