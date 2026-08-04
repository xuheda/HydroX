"""Evaluate a waypoint-tracking XLog against its recorded showcase route.

The report deliberately uses the simulator state stream and the exact route from
the showcase JSON.  It is useful for establishing a GNC baseline before a
residual-RL policy is enabled.

Example:
  python learning/evaluation/trajectory_report.py run.xlog \
      --showcase learning/showcases/ecaa9_figure_eight.json \
      --output learning/evaluation/figure_eight_report.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import numpy as np


THIS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(THIS_DIR.parent / "training"))
from xlog_dataset import TOPIC_SETPOINT, TOPIC_STATE, iter_records  # noqa: E402


def _float(value: float) -> float:
    return float(value)


def _metric(values: np.ndarray) -> dict[str, float]:
    return {
        "mean_m": _float(np.mean(values)),
        "rmse_m": _float(np.sqrt(np.mean(np.square(values)))),
        "median_m": _float(np.median(values)),
        "p95_m": _float(np.percentile(values, 95)),
        "max_m": _float(np.max(values)),
    }


def _load_waypoints(showcase_path: Path) -> tuple[np.ndarray, float]:
    showcase = json.loads(showcase_path.read_text(encoding="utf-8"))
    for action in (*showcase.get("actions", []), *showcase.get("acts", [])):
        if action.get("type") == "follow_waypoints":
            raw_waypoints = action["waypoints"]
            if raw_waypoints and isinstance(raw_waypoints[0], dict):
                raw_waypoints = [[point["x"], point["y"], point["z"]] for point in raw_waypoints]
            waypoints = np.asarray(raw_waypoints, dtype=np.float64)
            if waypoints.ndim != 2 or waypoints.shape[1] != 3:
                raise ValueError("follow_waypoints must contain [N, E, D] points")
            return waypoints, float(action.get("acceptance_radius_m", action.get("radius", 4.0)))
    raise ValueError(f"{showcase_path}: no follow_waypoints action")


def _nearest_route(actual: np.ndarray, route: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return horizontal distance, signed depth error, and planar route progress."""
    starts = route[:-1]
    deltas = route[1:] - starts
    planar = deltas[:, :2]
    planar_len_sq = np.sum(np.square(planar), axis=1)
    if np.any(planar_len_sq < 1.0e-9):
        raise ValueError("route has adjacent points with identical horizontal position")

    offset = actual[:, None, :2] - starts[None, :, :2]
    fractions = np.clip(np.sum(offset * planar[None, :, :], axis=2) / planar_len_sq[None, :], 0.0, 1.0)
    projected_ne = starts[None, :, :2] + fractions[:, :, None] * planar[None, :, :]
    planar_error = np.linalg.norm(actual[:, None, :2] - projected_ne, axis=2)
    chosen = np.argmin(planar_error, axis=1)
    row = np.arange(actual.shape[0])
    chosen_fraction = fractions[row, chosen]
    expected_depth = starts[chosen, 2] + chosen_fraction * deltas[chosen, 2]

    segment_lengths = np.sqrt(planar_len_sq)
    progress_prefix = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    progress = progress_prefix[chosen] + chosen_fraction * segment_lengths[chosen]
    return planar_error[row, chosen], actual[:, 2] - expected_depth, progress


def evaluate(log_path: Path, showcase_path: Path) -> dict[str, Any]:
    waypoints, acceptance_radius = _load_waypoints(showcase_path)
    state_records = []
    setpoint_records = []
    for record in iter_records(log_path):
        if record.topic == TOPIC_STATE:
            state_records.append(record)
        elif record.topic == TOPIC_SETPOINT:
            setpoint_records.append(record)
    if not state_records:
        raise ValueError(f"{log_path}: no vehicle-state records")

    first_target = waypoints[0]
    matching_targets = [record.timestamp_ns for record in setpoint_records
                        if np.linalg.norm(record.value["wp"] - first_target) <= 0.25]
    mission_start_ns = matching_targets[0] if matching_targets else state_records[0].timestamp_ns
    commanded_indices: list[int] = []
    for record in setpoint_records:
        if record.timestamp_ns < mission_start_ns:
            continue
        distances = np.linalg.norm(waypoints - record.value["wp"], axis=1)
        closest_index = int(np.argmin(distances))
        if distances[closest_index] <= 0.25:
            commanded_indices.append(closest_index)
    last_commanded_index = max(commanded_indices, default=0)
    state_times_ns = np.asarray([record.timestamp_ns for record in state_records], dtype=np.int64)
    start_index = int(np.searchsorted(state_times_ns, mission_start_ns, side="left"))
    states = np.asarray([record.value["eta"][:3] for record in state_records[start_index:]], dtype=np.float64)
    timestamps_ns = state_times_ns[start_index:]
    if len(states) < 2:
        raise ValueError(f"{log_path}: insufficient post-command state records")

    # The showcase route starts with the actual position at command acceptance;
    # the configured waypoints then define the commanded figure-eight.
    route = np.vstack((states[0], waypoints))
    # A figure-eight intersects at the centre, so nearest distance to the whole
    # route could incorrectly classify the start as a later crossing.  Restrict
    # error/progress metrics to the ordered prefix that GNC actually commanded.
    attempted_route = route[:last_commanded_index + 2]
    horizontal_error, depth_error, progress = _nearest_route(states, attempted_route)
    route_length = float(np.sum(np.linalg.norm(np.diff(route[:, :2], axis=0), axis=1)))
    attempted_length = float(np.sum(np.linalg.norm(np.diff(attempted_route[:, :2], axis=0), axis=1)))
    waypoint_minimums = [
        float(np.min(np.linalg.norm(states[:, :2] - waypoint[:2], axis=1)))
        for waypoint in waypoints
    ]
    elapsed_s = (timestamps_ns - timestamps_ns[0]) / 1.0e9
    stride = max(1, len(states) // 600)
    downsample = slice(None, None, stride)

    return {
        "source_xlog": str(log_path),
        "showcase": str(showcase_path),
        "mission_start_detected_from_setpoint": bool(matching_targets),
        "samples": int(len(states)),
        "duration_s": _float(elapsed_s[-1]),
        "acceptance_radius_m": acceptance_radius,
        "planned_horizontal_length_m": route_length,
        "attempted_horizontal_length_m": attempted_length,
        "last_commanded_waypoint_index": last_commanded_index,
        "commanded_waypoint_indices": sorted(set(commanded_indices)),
        "waypoints_completed_by_command_advance": last_commanded_index,
        "max_route_progress_m": _float(np.max(progress)),
        "max_route_progress_pct_of_attempted": _float(100.0 * np.max(progress) / attempted_length),
        "horizontal_error": _metric(horizontal_error),
        "absolute_depth_error": _metric(np.abs(depth_error)),
        "waypoint_min_horizontal_distance_m": waypoint_minimums,
        "note": "Error is against the ordered route prefix up to the last GNC-commanded waypoint.",
        "trajectory": {
            "planned_ned": route.tolist(),
            "attempted_route_ned": attempted_route.tolist(),
            "actual_ned": states[downsample].tolist(),
            "elapsed_s": elapsed_s[downsample].tolist(),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("xlog", type=Path)
    parser.add_argument("--showcase", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    report = evaluate(args.xlog, args.showcase)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items() if key != "trajectory"}, indent=2))


if __name__ == "__main__":
    main()
