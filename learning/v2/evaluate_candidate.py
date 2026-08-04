"""Independent fixed-seed evaluation for a frozen low-level v2 candidate."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

V2_ROOT = Path(__file__).resolve().parent
if str(V2_ROOT) not in sys.path:
    sys.path.insert(0, str(V2_ROOT))

from residual_env import (  # noqa: E402
    EcaA9LowLevelResidualEnv,
    EnvConfig,
    evaluate_policy,
)
from train_sac import (  # noqa: E402
    PriorRegularizedSAC,
    _comparison_metrics,
    _model_policy,
    _plot_comparison,
)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Evaluate a frozen ECA A9 low-level residual policy")
    parser.add_argument("model", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--episodes", type=int, default=12)
    parser.add_argument("--seed", type=int, default=20260730 + 50_000)
    parser.add_argument("--episode-s", type=float, default=20.0)
    parser.add_argument("--observation-noise-scale", type=float, default=0.25)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--shadow-executable", type=Path)
    parser.add_argument("--vehicle-parameters", type=Path)
    args = parser.parse_args()
    if args.episodes <= 0:
        raise ValueError("--episodes must be positive")
    if args.observation_noise_scale < 0.0:
        raise ValueError("--observation-noise-scale must be non-negative")
    defaults = EnvConfig()
    config = EnvConfig(
        episode_s=args.episode_s,
        observation_noise_scale=args.observation_noise_scale,
        domain_randomization=True,
        shadow_executable=str(
            args.shadow_executable or defaults.shadow_executable),
        vehicle_parameters=str(
            args.vehicle_parameters or defaults.vehicle_parameters),
        limits=defaults.limits,
    )
    model = PriorRegularizedSAC.load(str(args.model), device=args.device)
    seeds = [args.seed + index for index in range(args.episodes)]
    factory = lambda: EcaA9LowLevelResidualEnv(config)
    baseline, baseline_trajectories = evaluate_policy(
        factory, lambda _observation: np.zeros(2, dtype=np.float32), seeds)
    policy, policy_trajectories = evaluate_policy(
        factory, _model_policy(model), seeds)
    comparison = _comparison_metrics(baseline, policy)

    args.output.mkdir(parents=True, exist_ok=True)
    _plot_comparison(
        args.output,
        baseline_trajectories,
        policy_trajectories,
        filename="independent_baseline_vs_residual.png",
    )
    independent_plot = args.output / "independent_baseline_vs_residual.png"
    report = {
        "format": "hydrox.low-level-residual-independent-evaluation/v2",
        "model": str(args.model.resolve()),
        "model_sha256": _sha256(args.model),
        "episodes": args.episodes,
        "seeds": seeds,
        "episode_s": args.episode_s,
        "domain_randomization": True,
        "observation_noise_scale": args.observation_noise_scale,
        "baseline": baseline,
        "policy": policy,
        "comparison": comparison,
        "plot": independent_plot.name,
        "deployment_authority": "none",
    }
    report_path = args.output / "independent_evaluation.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
