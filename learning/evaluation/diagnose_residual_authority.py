"""Measure residual-action authority before spending more SAC training budget."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

TRAINING_DIR = Path(__file__).resolve().parents[1] / "training"
sys.path.insert(0, str(TRAINING_DIR))
from residual_env import EnvConfig, PinnEnsemble, ResidualAuvEnv, SafetyLimits, rollout  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pinn", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--episodes", type=int, default=6)
    parser.add_argument("--seed", type=int, default=20270728)
    args = parser.parse_args()
    ensemble = PinnEnsemble(sorted(args.pinn.glob("pinn_member_*.pt")))
    actions = {
        "zero": np.asarray((0.0, 0.0, 0.0), dtype=np.float32),
        "max_x": np.asarray((1.0, 0.0, 0.0), dtype=np.float32),
        "max_n_positive": np.asarray((0.0, 0.0, 1.0), dtype=np.float32),
        "max_n_negative": np.asarray((0.0, 0.0, -1.0), dtype=np.float32),
        "max_x_n": np.asarray((1.0, 0.0, 1.0), dtype=np.float32),
    }
    report: dict[str, object] = {"format": "hydrox.residual-authority-diagnostic/v1", "runs": {}}
    for blend in (0.10, 0.50):
        config = EnvConfig(safety=SafetyLimits(blend=blend))
        environment = ResidualAuvEnv(ensemble, config)
        values = {}
        for label, action in actions.items():
            values[label] = rollout(environment, lambda _obs, action=action: action, args.episodes, args.seed)
        report["runs"][str(blend)] = values
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
