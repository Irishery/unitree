"""Visually evaluate a trained SAC G1/DEX3 pickup policy in MuJoCo."""
import argparse
import time

import mujoco
import mujoco.viewer
from stable_baselines3 import SAC

from g1_mujoco.pick_env import CURRICULUM_STAGES, G1PickEnv


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, help="Path to SAC .zip checkpoint.")
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--physics-steps", type=int, default=10,
                        help="Use 10 for faithful visual contact physics.")
    parser.add_argument("--realtime", action="store_true",
                        help="Limit simulation to real time instead of running as fast as possible.")
    parser.add_argument("--stage", choices=CURRICULUM_STAGES, default="grasp",
                        help="Must match the curriculum stage used to train the checkpoint.")
    args = parser.parse_args()
    if args.episodes < 1 or args.physics_steps < 1:
        parser.error("--episodes and --physics-steps must be positive")

    env = G1PickEnv(physics_steps=args.physics_steps, stage=args.stage)
    policy = SAC.load(args.model, device="cpu")
    successes = 0
    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        for episode in range(args.episodes):
            observation, _ = env.reset(seed=args.seed + episode)
            reward_total = 0.0
            done = False
            while viewer.is_running() and not done:
                started = time.perf_counter()
                action, _ = policy.predict(observation, deterministic=True)
                observation, reward, terminated, truncated, info = env.step(action)
                reward_total += reward
                done = terminated or truncated
                viewer.sync()
                if args.realtime:
                    remaining = env.model.opt.timestep * args.physics_steps - (time.perf_counter() - started)
                    if remaining > 0:
                        time.sleep(remaining)
            success = info["lifted"] > 0.12
            successes += success
            print(f"episode={episode + 1}/{args.episodes} success={success} "
                  f"lifted={info['lifted']:.3f} contacts={info['contacts']} reward={reward_total:.2f}", flush=True)
            if not viewer.is_running():
                break
    print(f"success_rate={successes}/{args.episodes}")


if __name__ == "__main__":
    main()
