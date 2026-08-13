"""Train SAC on the headless MuJoCo G1 pickup environment."""
import argparse
import os
from pathlib import Path

from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

from g1_mujoco.pick_env import CURRICULUM_STAGES, G1PickEnv


def make_env(rank, seed, physics_steps, stage):
    """Top-level factory: required by SubprocVecEnv worker processes."""
    def _factory():
        env = G1PickEnv(physics_steps=physics_steps, stage=stage)
        env.reset(seed=seed + rank)
        return Monitor(env)
    return _factory


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timesteps", type=int, default=1_000_000)
    parser.add_argument("--output", default="/ws/models/g1_pick_sac")
    parser.add_argument("--tensorboard-log", default=None,
                        help="Optional directory for TensorBoard logs (requires tensorboard package).")
    parser.add_argument("--envs", type=int, default=min(8, os.cpu_count() or 1),
                        help="Number of parallel headless MuJoCo environments.")
    parser.add_argument("--physics-steps", type=int, default=5,
                        help="MuJoCo integration steps per policy action; 5 is the fast training default.")
    parser.add_argument("--checkpoint-every", type=int, default=100_000,
                        help="Save a checkpoint after this many aggregate environment steps; 0 disables it.")
    parser.add_argument("--device", default="auto", help="PyTorch device: auto, cpu, cuda, ...")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--stage", choices=CURRICULUM_STAGES, default="grasp",
                        help="Curriculum phase: grasp, approach, then full.")
    parser.add_argument("--load", default=None,
                        help="Optional .zip checkpoint from the preceding curriculum phase.")
    args = parser.parse_args()
    if args.envs < 1 or args.physics_steps < 1:
        parser.error("--envs and --physics-steps must be positive")
    # Do not oversubscribe CPUs: MuJoCo workers use the cores, PyTorch uses one.
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    os.environ.setdefault("MKL_NUM_THREADS", "1")
    env_fns = [make_env(rank, args.seed, args.physics_steps, args.stage) for rank in range(args.envs)]
    env = DummyVecEnv(env_fns) if args.envs == 1 else SubprocVecEnv(env_fns, start_method="forkserver")
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    callbacks = []
    if args.checkpoint_every:
        callbacks.append(CheckpointCallback(
            save_freq=max(args.checkpoint_every // args.envs, 1),
            save_path=str(output.parent / "checkpoints"), name_prefix=output.name))
    if args.load:
        model = SAC.load(args.load, env=env, device=args.device)
        model.verbose = 1
        model.tensorboard_log = args.tensorboard_log
    else:
        model = SAC("MlpPolicy", env, learning_rate=3e-4, buffer_size=500_000,
                    learning_starts=max(5_000, 1_000 * args.envs), batch_size=512,
                    train_freq=(1, "step"), gradient_steps=1, verbose=1,
                    tensorboard_log=args.tensorboard_log, device=args.device, seed=args.seed)
    model.learn(total_timesteps=args.timesteps, callback=callbacks or None, reset_num_timesteps=not bool(args.load))
    model.save(args.output)
    env.close()


if __name__ == "__main__":
    main()
