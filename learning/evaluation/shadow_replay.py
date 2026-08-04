"""Replay a frozen residual policy on recorded XLog-derived observations.

This is a no-authority check: it records the policy's proposed and safety
filtered Δtau against historical base wrenches.  It does not alter a log,
invoke DDS, or command SITL.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

TRAINING_DIR = Path(__file__).resolve().parents[1] / "training"
sys.path.insert(0, str(TRAINING_DIR))
from residual_env import EnvConfig, PinnEnsemble, ResidualAuvEnv, SafetyLimits  # noqa: E402


def ensemble_uncertainty(ensemble: PinnEnsemble, eta: np.ndarray, nu: np.ndarray,
                         tau: np.ndarray, batch_size: int = 2048) -> np.ndarray:
    values: list[np.ndarray] = []
    with torch.inference_mode():
        for start in range(0, len(eta), batch_size):
            end = min(start + batch_size, len(eta))
            eta_t = torch.as_tensor(eta[start:end], dtype=torch.float32, device=ensemble.device)
            nu_t = torch.as_tensor(nu[start:end], dtype=torch.float32, device=ensemble.device)
            tau_t = torch.as_tensor(tau[start:end], dtype=torch.float32, device=ensemble.device)
            accelerations = torch.stack([model(eta_t, nu_t, tau_t)[0] for model in ensemble.models])
            values.append(accelerations.std(dim=0, correction=0)[:, [0, 4, 5]].cpu().numpy())
    return np.concatenate(values).astype(np.float64)


def percentile_abs(values: np.ndarray, percentile: float = 99.0) -> list[float]:
    return np.percentile(np.abs(values), percentile, axis=0).astype(float).tolist()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--pinn", required=True, type=Path)
    parser.add_argument("--candidate", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--blend", type=float, default=0.10)
    parser.add_argument("--policy-hz", type=float, default=20.0)
    args = parser.parse_args()
    if not 0.0 < args.blend <= 1.0 or args.policy_hz <= 0.0:
        raise ValueError("blend and policy-hz must be positive")

    data = np.load(args.dataset, allow_pickle=False)
    feature_index = {str(name): index for index, name in enumerate(data["feature_names"])}
    needed = ("depth_error_m", "surge_error_mps", "yaw_rate_error_radps", "sin_heading_error", "cos_heading_error")
    if any(name not in feature_index for name in needed):
        raise ValueError("dataset observation contract is missing a required control field")
    ensemble = PinnEnsemble(sorted(args.pinn.glob("pinn_member_*.pt")))
    safety = SafetyLimits(blend=args.blend)
    dummy = DummyVecEnv([lambda: ResidualAuvEnv(ensemble, EnvConfig(safety=safety))])
    normalizer = VecNormalize.load(str(args.candidate / "vecnormalize.pkl"), dummy)
    normalizer.training = False
    normalizer.norm_reward = False
    policy_path = args.candidate / "best_model" / "best_model.zip"
    if not policy_path.is_file():
        raise FileNotFoundError(policy_path)
    policy = SAC.load(str(policy_path), device="cpu")
    eta = data["eta"].astype(np.float64)
    nu = data["nu"].astype(np.float64)
    tau = data["tau_base"].astype(np.float64)
    uncertainty = ensemble_uncertainty(ensemble, eta, nu, tau)
    observations = data["observation"]
    source_run = data["source_run"]
    dt = data["dt"].astype(np.float64)
    proposed = np.zeros((len(dt), 3), dtype=np.float64)
    applied = np.zeros_like(proposed)
    update_count = np.zeros(len(dt), dtype=bool)
    limits = np.asarray(safety.max_delta, dtype=np.float64)
    normalizers = np.asarray((280.0, 35.0, 125.0), dtype=np.float64)
    uncertainty_normalizers = np.asarray((0.6, 0.3, 0.3), dtype=np.float64)
    previous_delta = np.zeros(3, dtype=np.float64)
    held_action = np.zeros(3, dtype=np.float64)
    elapsed = 1.0 / args.policy_hz
    for index in range(len(dt)):
        if index == 0 or source_run[index] != source_run[index - 1]:
            previous_delta.fill(0.0)
            held_action.fill(0.0)
            elapsed = 1.0 / args.policy_hz
        if elapsed >= 1.0 / args.policy_hz:
            raw = np.concatenate((
                nu[index],
                observations[index, [feature_index[name] for name in needed]],
                tau[index, [0, 4, 5]] / normalizers,
                previous_delta / (limits * safety.blend),
                np.clip(uncertainty[index] / uncertainty_normalizers, 0.0, 5.0),
            ))
            raw = np.clip(raw, -20.0, 20.0).astype(np.float32)
            held_action = policy.predict(normalizer.normalize_obs(raw[None, :]), deterministic=True)[0][0]
            update_count[index] = True
            elapsed = 0.0
        previous_delta = safety.requested(held_action, previous_delta, float(dt[index]))
        proposed[index] = held_action
        applied[index] = previous_delta
        elapsed += float(dt[index])

    runs = []
    for run_id in np.unique(source_run):
        selected = source_run == run_id
        runs.append({
            "source_run": int(run_id), "samples": int(selected.sum()),
            "policy_updates": int(update_count[selected].sum()),
            "proposed_action_abs_p99": percentile_abs(proposed[selected]),
            "applied_delta_abs_p99": percentile_abs(applied[selected]),
            "applied_delta_abs_max": np.max(np.abs(applied[selected]), axis=0).astype(float).tolist(),
        })
    report = {
        "format": "hydrox.residual-shadow-replay/v1", "authority": "none",
        "dataset": str(args.dataset), "pinn": str(args.pinn), "candidate": str(policy_path),
        "policy_hz": args.policy_hz, "safety": {"blend": safety.blend, "max_delta": list(safety.max_delta),
                   "max_rate": list(safety.max_rate)},
        "all_proposed_actions_finite": bool(np.isfinite(proposed).all()),
        "all_proposed_actions_in_range": bool(np.all(np.abs(proposed) <= 1.000001)),
        "runs": runs,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
