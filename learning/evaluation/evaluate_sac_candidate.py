"""Re-evaluate final and callback-best SAC policies on identical fixed seeds."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from stable_baselines3 import SAC
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

TRAINING_DIR = Path(__file__).resolve().parents[1] / "training"
sys.path.insert(0, str(TRAINING_DIR))
from residual_env import EnvConfig, PinnEnsemble, ResidualAuvEnv, rollout  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pinn", required=True, type=Path)
    parser.add_argument("--candidate", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--seed", type=int, default=20270727)
    args = parser.parse_args()
    ensemble = PinnEnsemble(sorted(args.pinn.glob("pinn_member_*.pt")))
    config = EnvConfig()
    dummy = DummyVecEnv([lambda: ResidualAuvEnv(ensemble, config)])
    normalized = VecNormalize.load(str(args.candidate / "vecnormalize.pkl"), dummy)
    normalized.training = False
    normalized.norm_reward = False
    raw = ResidualAuvEnv(ensemble, config)
    baseline = rollout(raw, lambda _obs: np.zeros(3, dtype=np.float32), args.episodes, args.seed)
    policies = {
        "final": args.candidate / "residual_sac.zip",
        "best_callback": args.candidate / "best_model" / "best_model.zip",
    }
    results = {}
    for name, path in policies.items():
        if not path.is_file():
            continue
        model = SAC.load(str(path), device="cpu")
        result = rollout(
            raw,
            lambda observation, model=model: model.predict(
                normalized.normalize_obs(observation[None, :]), deterministic=True)[0][0],
            args.episodes,
            args.seed,
        )
        result["tracking_error_improvement_m"] = baseline["mean_tracking_error_m"] - result["mean_tracking_error_m"]
        result["hard_termination_not_worse"] = result["hard_termination_rate"] <= baseline["hard_termination_rate"]
        results[name] = result
    report = {"format": "hydrox.sac-fixed-seed-evaluation/v1", "episodes": args.episodes,
              "seed": args.seed, "baseline": baseline, "policies": results}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
