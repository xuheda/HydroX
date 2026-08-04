"""Train a guarded residual-SAC candidate in the frozen PINN ensemble.

This command produces a Python-only pilot artifact.  It does not export ONNX,
modify the C++ runtime, or enable learned authority in SITL.
"""
from __future__ import annotations

import argparse
import json
import random
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback, CallbackList, EvalCallback
from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor, VecNormalize

from residual_env import EnvConfig, OBSERVATION_NAMES, PinnEnsemble, ResidualAuvEnv, SafetyLimits, rollout


class ProgressCheckpointCallback(BaseCallback):
    """Persist all state needed to evaluate or resume a local Shadow-only run."""

    def __init__(self, output: Path, save_freq: int) -> None:
        super().__init__()
        self.output = output / "progress"
        self.save_freq = save_freq

    def _on_step(self) -> bool:
        if self.save_freq <= 0 or self.num_timesteps % self.save_freq != 0:
            return True
        self.output.mkdir(parents=True, exist_ok=True)
        stem = self.output / f"step_{self.num_timesteps:08d}"
        self.model.save(str(stem))
        self.model.save_replay_buffer(str(stem.with_suffix(".replay_buffer.pkl")))
        environment = self.model.get_env()
        if not isinstance(environment, VecNormalize):
            raise TypeError("residual SAC training requires VecNormalize")
        environment.save(str(stem.with_suffix(".vecnormalize.pkl")))
        return True


def checkpoint_paths(directory: Path) -> list[Path]:
    paths = sorted(directory.glob("pinn_member_*.pt"))
    if not paths:
        raise FileNotFoundError(f"{directory}: no pinn_member_*.pt checkpoints")
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(description="Train a HydroX PINN-informed residual SAC pilot")
    parser.add_argument("--pinn", required=True, type=Path, help="PINN ensemble directory")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timesteps", type=int, default=60000)
    parser.add_argument("--seed", type=int, default=20260726)
    parser.add_argument("--eval-episodes", type=int, default=20)
    parser.add_argument("--callback-eval-episodes", type=int, default=3)
    parser.add_argument("--train-blend", type=float, default=0.10,
                        help="residual authority used in the training curriculum")
    parser.add_argument("--evaluation-blend", type=float, default=None,
                        help="guarded authority used for final fixed-seed evaluation; defaults to train blend")
    parser.add_argument("--device", default="cpu", help="PyTorch device for the frozen PINN ensemble")
    parser.add_argument("--hydrox-shadow-exe", type=Path,
                        help="offline hydrox_control_shadow executable; requires --vehicle-params")
    parser.add_argument("--vehicle-params", type=Path,
                        help="exact EcaA9 parameter JSON used by SITL; requires --hydrox-shadow-exe")
    parser.add_argument("--save-every", type=int, default=5000,
                        help="save complete local training state every N environment steps; 0 disables")
    parser.add_argument("--resume-model", type=Path,
                        help="previous SAC .zip checkpoint; requires --resume-vecnormalize")
    parser.add_argument("--resume-vecnormalize", type=Path,
                        help="VecNormalize checkpoint paired with --resume-model")
    parser.add_argument("--resume-replay-buffer", type=Path,
                        help="optional replay-buffer checkpoint paired with --resume-model")
    args = parser.parse_args()
    if args.timesteps <= 0:
        raise ValueError("--timesteps must be positive")
    if not 0.0 < args.train_blend <= 1.0:
        raise ValueError("--train-blend must be in (0, 1]")
    evaluation_blend = args.train_blend if args.evaluation_blend is None else args.evaluation_blend
    if not 0.0 < evaluation_blend <= 1.0:
        raise ValueError("--evaluation-blend must be in (0, 1]")
    if (args.hydrox_shadow_exe is None) != (args.vehicle_params is None):
        raise ValueError("--hydrox-shadow-exe and --vehicle-params must be supplied together")
    if args.save_every < 0:
        raise ValueError("--save-every must be non-negative")
    if (args.resume_model is None) != (args.resume_vecnormalize is None):
        raise ValueError("--resume-model and --resume-vecnormalize must be supplied together")

    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    output = args.output
    output.mkdir(parents=True, exist_ok=True)
    pinn_paths = checkpoint_paths(args.pinn)
    ensemble = PinnEnsemble(pinn_paths, device=args.device)
    control_paths = {
        "hydrox_shadow_exe": str(args.hydrox_shadow_exe) if args.hydrox_shadow_exe else None,
        "vehicle_params_path": str(args.vehicle_params) if args.vehicle_params else None,
    }
    train_config = EnvConfig(safety=SafetyLimits(blend=args.train_blend), **control_paths)
    evaluation_config = EnvConfig(safety=SafetyLimits(blend=evaluation_blend), **control_paths)

    def make_env(rank: int):
        def factory():
            return ResidualAuvEnv(ensemble, train_config)
        return factory

    raw_train_env = VecMonitor(DummyVecEnv([make_env(0)]))
    train_env = (VecNormalize.load(str(args.resume_vecnormalize), raw_train_env)
                 if args.resume_vecnormalize else
                 VecNormalize(raw_train_env, norm_obs=True, norm_reward=False, clip_obs=10.0))
    train_env.training = True
    train_env.norm_reward = False
    eval_env = VecNormalize(VecMonitor(DummyVecEnv([make_env(1)])), training=False, norm_obs=True,
                            norm_reward=False, clip_obs=10.0)
    evaluation_dir = output / "evaluations"
    callback = EvalCallback(eval_env, best_model_save_path=str(output / "best_model"),
                            log_path=str(evaluation_dir), eval_freq=5000,
                            n_eval_episodes=max(1, args.callback_eval_episodes), deterministic=True,
                            render=False)
    if args.resume_model:
        model = SAC.load(str(args.resume_model), env=train_env, device="cpu")
        if args.resume_replay_buffer:
            model.load_replay_buffer(str(args.resume_replay_buffer))
    else:
        model = SAC(
            "MlpPolicy", train_env, seed=args.seed, device="cpu", verbose=1,
            learning_rate=3.0e-4, buffer_size=150_000, learning_starts=2_000,
            batch_size=256, tau=0.005, gamma=0.99, train_freq=1, gradient_steps=1,
            policy_kwargs={"net_arch": [256, 256]},
        )
    callbacks = [callback]
    if args.save_every:
        callbacks.append(ProgressCheckpointCallback(output, args.save_every))
    model.learn(total_timesteps=args.timesteps, callback=CallbackList(callbacks), progress_bar=False,
                reset_num_timesteps=args.resume_model is None)
    model.save(str(output / "residual_sac"))
    train_env.save(str(output / "vecnormalize.pkl"))

    raw_eval = ResidualAuvEnv(ensemble, evaluation_config)
    baseline = rollout(raw_eval, lambda _observation: np.zeros(3, dtype=np.float32), args.eval_episodes, args.seed + 10_000)
    policy = rollout(raw_eval, lambda observation: model.predict(train_env.normalize_obs(observation[None, :]), deterministic=True)[0][0],
                     args.eval_episodes, args.seed + 10_000)
    improvement = baseline["mean_tracking_error_m"] - policy["mean_tracking_error_m"]
    metrics = {
        "format": "hydrox.residual-sac-pilot/v1",
        "phase": "2_rl_sitl_surrogate_only",
        "deployment_authority": "none",
        "pinn_checkpoints": [str(path) for path in pinn_paths],
        "seed": args.seed,
        "timesteps": args.timesteps,
        "resumed_from": str(args.resume_model) if args.resume_model else None,
        "policy_hz": 20.0,
        "observation_names": list(OBSERVATION_NAMES),
        "action_axes": ["X", "M", "N"],
        "control_stack": "hydrox_cpp_shadow" if args.hydrox_shadow_exe else "python_surrogate",
        "safety": {
            "train_blend": train_config.safety.blend,
            "evaluation_blend": evaluation_config.safety.blend,
            "max_delta": list(evaluation_config.safety.max_delta),
            "max_rate": list(evaluation_config.safety.max_rate),
        },
        "baseline": baseline,
        "policy": policy,
        "tracking_error_improvement_m": improvement,
        "gates": {
            "multi_condition_pinn_validation": True,
            "unseen_shear_vortex_validation": True,
            "SITL_shadow_replay": False,
            "guarded_SITL_authority_allowed": False,
        },
    }
    (output / "evaluation.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    (output / "limits.json").write_text(json.dumps(metrics["safety"], indent=2), encoding="utf-8")
    (output / "normalization.json").write_text(json.dumps({
        "format": "hydrox.residual-observation/v1", "fields": list(OBSERVATION_NAMES),
        "action_axes": ["X", "M", "N"], "vecnormalize": "vecnormalize.pkl",
        "note": "Python pilot only; not a deployable C++ observation contract.",
    }, indent=2), encoding="utf-8")
    raw_eval.close()
    train_env.close()
    eval_env.close()
    print(json.dumps(metrics, indent=2))


if __name__ == "__main__":
    main()
