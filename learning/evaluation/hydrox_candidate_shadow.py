"""Replay a frozen SAC candidate through the offline C++ HydroX control stack.

Historical states and setpoints are replayed in time order.  This checks the
learned policy's wrench/actuator envelope and C++ safety-filter behavior; it
does not claim a closed-loop tracking improvement because historical states are
not allowed to change in this Shadow pass.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import torch
from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

TRAINING_DIR = Path(__file__).resolve().parents[1] / "training"
sys.path.insert(0, str(TRAINING_DIR))
from hydrox_shadow_client import INPUT_HEADER  # noqa: E402
from residual_env import EnvConfig, PinnEnsemble, ResidualAuvEnv, SafetyLimits  # noqa: E402


def require_contract(data: np.lib.npyio.NpzFile) -> None:
    required = {"observation", "eta", "nu", "depth_m", "setpoint", "gnc_mode", "reset_controller",
                "dt", "tau_base", "source_run", "feature_names"}
    missing = sorted(required.difference(data.files))
    if missing:
        raise ValueError("dataset lacks v2 control-shadow fields: " + ", ".join(missing))


def uncertainty(ensemble: PinnEnsemble, eta: np.ndarray, nu: np.ndarray, tau: np.ndarray) -> np.ndarray:
    result: list[np.ndarray] = []
    with torch.inference_mode():
        for start in range(0, len(eta), 2048):
            end = min(start + 2048, len(eta))
            eta_t = torch.as_tensor(eta[start:end], dtype=torch.float32, device=ensemble.device)
            nu_t = torch.as_tensor(nu[start:end], dtype=torch.float32, device=ensemble.device)
            tau_t = torch.as_tensor(tau[start:end], dtype=torch.float32, device=ensemble.device)
            values = torch.stack([model(eta_t, nu_t, tau_t)[0] for model in ensemble.models])
            result.append(values.std(dim=0, correction=0)[:, [0, 4, 5]].cpu().numpy())
    return np.concatenate(result).astype(np.float64)


def p99(values: np.ndarray) -> list[float]:
    return np.percentile(np.abs(values), 99.0, axis=0).astype(float).tolist()


def write_batch_input(data: np.lib.npyio.NpzFile, actions: np.ndarray, path: Path) -> None:
    """Write precomputed policy actions for one efficient C++ stateful replay."""
    eta = data["eta"].astype(np.float64)
    nu = data["nu"].astype(np.float64)
    setpoint = data["setpoint"].astype(np.float64)
    rows = np.column_stack((
        data["reset_controller"].astype(np.float64), data["dt"].astype(np.float64),
        data["gnc_mode"].astype(np.float64), eta, nu, data["depth_m"].astype(np.float64),
        setpoint[:, 0:3], setpoint[:, 7], setpoint[:, 3:7], actions,
        np.ones((len(actions), 2), dtype=np.float64),
    ))
    np.savetxt(path, rows, delimiter=",", header=INPUT_HEADER, comments="", fmt="%.17g")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--pinn", type=Path, required=True)
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--shadow-exe", type=Path, required=True)
    parser.add_argument("--vehicle-params", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--blend", type=float, default=0.10)
    parser.add_argument("--policy-hz", type=float, default=20.0)
    parser.add_argument("--device", default="cuda", help="device for frozen PINN ensemble inference")
    parser.add_argument("--source-run", type=int,
                        help="replay one complete source run (useful for a fast per-condition safety gate)")
    args = parser.parse_args()
    if not 0.0 < args.blend <= 1.0 or args.policy_hz <= 0.0:
        raise ValueError("blend and policy-hz must be positive")
    data = np.load(args.dataset, allow_pickle=False)
    require_contract(data)
    if args.source_run is not None:
        selected = data["source_run"] == args.source_run
        if not np.any(selected):
            raise ValueError(f"source run {args.source_run} is not present")
        # Every saved source run begins at an explicit controller reset, so the
        # selected sequence remains a valid independent C++ replay segment.
        data = {
            key: (data[key][selected] if data[key].ndim >= 1 and data[key].shape[0] == len(selected) else data[key])
            for key in data.files
        }
    checkpoints = sorted(args.pinn.glob("pinn_member_*.pt"))
    ensemble = PinnEnsemble(checkpoints, device=args.device)
    safety = SafetyLimits(blend=args.blend)
    dummy = DummyVecEnv([lambda: ResidualAuvEnv(ensemble, EnvConfig(safety=safety))])
    normalizer_path = args.candidate / "vecnormalize.pkl"
    policy_path = args.candidate / "best_model" / "best_model.zip"
    if not normalizer_path.is_file() or not policy_path.is_file():
        raise FileNotFoundError("candidate requires vecnormalize.pkl and best_model/best_model.zip")
    normalizer = VecNormalize.load(str(normalizer_path), dummy)
    normalizer.training = False
    normalizer.norm_reward = False
    policy = SAC.load(str(policy_path), device="cpu")
    names = {str(name): index for index, name in enumerate(data["feature_names"])}
    needed = ("depth_error_m", "surge_error_mps", "yaw_rate_error_radps", "sin_heading_error", "cos_heading_error")
    if any(name not in names for name in needed):
        raise ValueError("dataset lacks policy error features")
    eta = data["eta"].astype(np.float64)
    nu = data["nu"].astype(np.float64)
    dt = data["dt"].astype(np.float64)
    source_run = data["source_run"]
    logged_tau = data["tau_base"].astype(np.float64)
    model_std = uncertainty(ensemble, eta, nu, logged_tau)
    proposed = np.zeros((len(dt), 3), dtype=np.float64)
    applied = np.zeros_like(proposed)
    policy_update = np.zeros(len(dt), dtype=bool)
    held_action = np.zeros(3, dtype=np.float64)
    previous_delta = np.zeros(3, dtype=np.float64)
    previous_base = np.zeros(3, dtype=np.float64)
    elapsed = 1.0 / args.policy_hz
    try:
        for index in range(len(dt)):
            reset = bool(data["reset_controller"][index])
            if reset:
                held_action.fill(0.0)
                previous_delta.fill(0.0)
                previous_base.fill(0.0)
                elapsed = 1.0 / args.policy_hz
            if elapsed >= 1.0 / args.policy_hz:
                raw = np.concatenate((
                    nu[index], data["observation"][index, [names[name] for name in needed]],
                    previous_base / np.asarray((280.0, 35.0, 125.0)),
                    previous_delta / (np.asarray(safety.max_delta) * safety.blend),
                    np.clip(model_std[index] / np.asarray((0.6, 0.3, 0.3)), 0.0, 5.0),
                )).astype(np.float32)
                held_action = policy.predict(normalizer.normalize_obs(raw[None, :]), deterministic=True)[0][0]
                policy_update[index] = True
                elapsed = 0.0
            proposed[index] = held_action
            # This is mathematically the same clamp/scale/rate rule as the
            # shared C++ filter.  The following batch replay remains the source
            # of truth and reports any mismatch explicitly.
            previous_delta = safety.requested(held_action, previous_delta, float(dt[index]))
            applied[index] = previous_delta
            previous_base = logged_tau[index, [0, 4, 5]]
            elapsed += float(dt[index])
    finally:
        dummy.close()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    input_path = args.output.with_suffix(".input.csv")
    replay_path = args.output.with_suffix(".output.csv")
    write_batch_input(data, proposed, input_path)
    subprocess.run([
        str(args.shadow_exe), "--input", str(input_path), "--output", str(replay_path),
        "--vehicle", "EcaA9", "--vehicle-params", str(args.vehicle_params),
        "--blend", f"{args.blend:.17g}", "--max-delta", "60,8,18", "--max-rate", "120,16,36",
    ], check=True)
    replay = np.loadtxt(replay_path, dtype=np.float64, delimiter=",", skiprows=1)
    if replay.ndim == 1:
        replay = replay[None, :]
    if replay.shape != (len(dt), 28) or not np.isfinite(replay).all():
        raise ValueError(f"invalid C++ batch replay output shape {replay.shape}")
    base = replay[:, 1:4]
    cxx_applied = replay[:, 7:10]
    final_actuator = replay[:, 19:27]
    def report_for(selected: np.ndarray) -> dict[str, object]:
        return {
            "samples": int(selected.sum()), "policy_updates": int(policy_update[selected].sum()),
            "proposed_action_abs_p99": p99(proposed[selected]),
            "applied_delta_abs_p99": p99(applied[selected]),
            "final_actuator_abs_p99": p99(final_actuator[selected]),
            "final_actuator_abs_max": np.max(np.abs(final_actuator[selected]), axis=0).astype(float).tolist(),
            "cxx_python_delta_rmse": np.sqrt(np.mean(
                np.square(cxx_applied[selected] - applied[selected]), axis=0)).astype(float).tolist(),
            "baseline_wrench_rmse_vs_log": np.sqrt(np.mean(
                np.square(base[selected] - logged_tau[selected][:, [0, 4, 5]]), axis=0)).astype(float).tolist(),
        }
    report = {
        "format": "hydrox.candidate-control-shadow/v1", "authority": "none",
        "scope": "historical-state actuator-envelope replay; not a closed-loop tracking claim",
        "dataset": str(args.dataset), "candidate": str(policy_path), "pinn": str(args.pinn),
        "policy_hz": args.policy_hz, "safety": {"blend": args.blend,
                    "max_delta": list(safety.max_delta), "max_rate": list(safety.max_rate)},
        "all_actions_finite": bool(np.isfinite(proposed).all()),
        "all_actions_in_range": bool(np.all(np.abs(proposed) <= 1.000001)),
        "all": report_for(np.ones(len(dt), dtype=bool)),
        "per_source_run": {str(int(run)): report_for(source_run == run) for run in np.unique(source_run)},
        "next_gate": "requires substantial closed-loop improvement and non-saturating outputs before SITL testing",
    }
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
