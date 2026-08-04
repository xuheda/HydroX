"""Validate logged EcaA9 control against the real offline HydroX control stack.

The replay uses all-zero residual actions.  Its purpose is to establish whether
the recorded XLog state/setpoint sequence can reproduce the original GNC and
allocator outputs before any learned candidate is considered.  It does not
launch SITL, connect to DDS/UE, or send actuator commands.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

TRAINING_DIR = Path(__file__).resolve().parents[1] / "training"
sys.path.insert(0, str(TRAINING_DIR))
from hydrox_shadow_client import INPUT_HEADER


SETPOINT_COLUMNS = (
    "depth_ref", "heading_ref", "surge_ref", "yaw_rate_ref",
    "wp_n", "wp_e", "wp_d", "use_yaw_rate_ref",
)


def require_contract(data: np.lib.npyio.NpzFile) -> None:
    needed = {"eta", "nu", "depth_m", "setpoint", "gnc_mode", "reset_controller", "dt", "tau_base", "actuator", "source_run"}
    missing = sorted(needed.difference(data.files))
    if missing:
        raise ValueError(
            "dataset lacks the v2 control-shadow contract: " + ", ".join(missing) +
            ". Rebuild it with learning/training/build_collection_dataset.py."
        )
    if data["setpoint"].shape[1:] != (8,):
        raise ValueError("setpoint must have columns " + ", ".join(SETPOINT_COLUMNS))


def rmse(values: np.ndarray) -> list[float]:
    return np.sqrt(np.mean(np.square(values), axis=0)).astype(float).tolist()


def p99_abs(values: np.ndarray) -> list[float]:
    return np.percentile(np.abs(values), 99.0, axis=0).astype(float).tolist()


def write_input(data: np.lib.npyio.NpzFile, path: Path) -> None:
    eta = data["eta"].astype(np.float64)
    nu = data["nu"].astype(np.float64)
    depth = data["depth_m"].astype(np.float64)
    setpoint = data["setpoint"].astype(np.float64)
    rows = np.column_stack((
        data["reset_controller"].astype(np.float64), data["dt"].astype(np.float64),
        data["gnc_mode"].astype(np.float64), eta, nu, depth, setpoint[:, 0:3],
        setpoint[:, 7], setpoint[:, 3:7],
        np.zeros((len(eta), 3), dtype=np.float64), np.ones((len(eta), 2), dtype=np.float64),
    ))
    np.savetxt(path, rows, delimiter=",", header=INPUT_HEADER, comments="", fmt="%.17g")


def summary_for(selected: np.ndarray, actual_tau: np.ndarray, actual_actuator: np.ndarray,
                replay: np.ndarray) -> dict[str, object]:
    tau_error = replay[selected, 1:4] - actual_tau[selected][:, [0, 4, 5]]
    actuator_error = replay[selected, 10:18] - actual_actuator[selected]
    return {
        "samples": int(selected.sum()),
        "base_wrench_rmse": rmse(tau_error),
        "base_wrench_abs_p99": p99_abs(tau_error),
        "base_actuator_rmse": rmse(actuator_error),
        "base_actuator_abs_p99": p99_abs(actuator_error),
        "zero_residual_delta_abs_max": float(np.max(np.abs(replay[selected, 7:10]))),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--shadow-exe", type=Path, required=True)
    parser.add_argument("--vehicle-params", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True,
                        help="report JSON path; adjacent CSV files remain local replay artifacts")
    parser.add_argument("--vehicle", default="EcaA9")
    args = parser.parse_args()
    if not args.shadow_exe.is_file():
        raise FileNotFoundError(args.shadow_exe)
    if not args.vehicle_params.is_file():
        raise FileNotFoundError(args.vehicle_params)
    data = np.load(args.dataset, allow_pickle=False)
    require_contract(data)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    input_path = args.output.with_suffix(".input.csv")
    output_path = args.output.with_suffix(".output.csv")
    write_input(data, input_path)
    subprocess.run([
        str(args.shadow_exe), "--input", str(input_path), "--output", str(output_path),
        "--vehicle", args.vehicle, "--vehicle-params", str(args.vehicle_params),
        "--blend", "0.10", "--max-delta", "60,8,18", "--max-rate", "120,16,36",
    ], check=True)
    replay = np.loadtxt(output_path, dtype=np.float64, delimiter=",", skiprows=1)
    if replay.ndim == 1:
        replay = replay[None, :]
    if replay.shape != (len(data["dt"]), 28):
        raise ValueError(f"unexpected C++ replay shape {replay.shape}")
    if not np.isfinite(replay).all():
        raise ValueError("C++ replay contains non-finite values")
    tau_base = data["tau_base"].astype(np.float64)
    actuator = data["actuator"].astype(np.float64)
    source_run = data["source_run"]
    per_run = {
        str(int(run)): summary_for(source_run == run, tau_base, actuator, replay)
        for run in np.unique(source_run)
    }
    report = {
        "format": "hydrox.control-shadow-replay/v1",
        "authority": "none",
        "residual_action": "zero",
        "dataset": str(args.dataset),
        "shadow_executable": str(args.shadow_exe),
        "vehicle_params": str(args.vehicle_params),
        "samples": int(len(replay)),
        "all": summary_for(np.ones(len(replay), dtype=bool), tau_base, actuator, replay),
        "per_source_run": per_run,
        "notes": [
            "A non-zero reproduction error can arise from controller reset events that legacy XLog did not record.",
            "This report is a prerequisite diagnostic only; it grants no learned-control authority.",
        ],
    }
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
