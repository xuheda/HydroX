"""Evaluate a frozen PINN ensemble per complete XLog run (no temporal leakage)."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

TRAINING_DIR = Path(__file__).resolve().parents[1] / "training"
sys.path.insert(0, str(TRAINING_DIR))
from pinn_residual import ModelConfig, axis_mask  # noqa: E402


def load_member(path: Path, device: torch.device):
    checkpoint = torch.load(path, map_location=device, weights_only=False)
    raw = checkpoint["config"]
    config = ModelConfig(
        axes=tuple(raw["axes"]), mass_kg=float(raw["mass_kg"]),
        pitch_inertia_kg_m2=float(raw["pitch_inertia_kg_m2"]),
        yaw_inertia_kg_m2=float(raw["yaw_inertia_kg_m2"]),
        residual_force_limits=tuple(float(value) for value in raw["residual_force_limits"]),
        width=int(raw["width"]), depth=int(raw["depth"]),
    )
    model = config.build(checkpoint["input_mean"], checkpoint["input_std"])
    model.load_state_dict(checkpoint["model_state"])
    return model.to(device).eval(), config


def metric(prediction: torch.Tensor, target: torch.Tensor, mask: torch.Tensor) -> float:
    return float((((prediction - target).square() * mask).sum(dim=-1) / mask.sum()).mean().cpu())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--pinn", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--device", default="cpu")
    args = parser.parse_args()
    device = torch.device(args.device)
    paths = sorted(args.pinn.glob("pinn_member_*.pt"))
    if not paths:
        raise FileNotFoundError(f"{args.pinn}: no PINN member checkpoints")
    loaded = [load_member(path, device) for path in paths]
    models = [pair[0] for pair in loaded]
    config = loaded[0][1]
    mask = axis_mask(config.axes).to(device)
    data = np.load(args.dataset, allow_pickle=False)
    eta = torch.as_tensor(data["eta"], dtype=torch.float32, device=device)
    nu = torch.as_tensor(data["nu"], dtype=torch.float32, device=device)
    tau = torch.as_tensor(data["tau_base"], dtype=torch.float32, device=device)
    dt = torch.as_tensor(data["dt"], dtype=torch.float32, device=device).unsqueeze(-1)
    target = (torch.as_tensor(data["next_nu"], dtype=torch.float32, device=device) - nu) / dt.clamp_min(1.0e-4)
    with torch.inference_mode():
        prediction = torch.stack([model(eta, nu, tau)[0] for model in models]).mean(dim=0)
        baseline = models[0].prior(nu, tau, torch.zeros_like(tau))
    runs = []
    for run_id in np.unique(data["source_run"]):
        index = torch.as_tensor(np.flatnonzero(data["source_run"] == run_id), device=device)
        model_mse = metric(prediction.index_select(0, index), target.index_select(0, index), mask)
        baseline_mse = metric(baseline.index_select(0, index), target.index_select(0, index), mask)
        runs.append({"source_run": int(run_id), "samples": int(len(index)), "model_mse": model_mse,
                     "baseline_mse": baseline_mse, "improvement_factor": baseline_mse / max(model_mse, 1.0e-12)})
    report = {"format": "hydrox.pinn-per-run-evaluation/v1", "dataset": str(args.dataset),
              "pinn": str(args.pinn), "active_axes": list(config.axes), "runs": runs}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
