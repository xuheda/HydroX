"""Build a multi-run PINN/RL dataset only from SHA-verified collection results."""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

from xlog_dataset import build_dataset


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_verified_runs(results_path: Path, expected_runs: int | None) -> list[dict[str, Any]]:
    records = json.loads(results_path.read_text(encoding="utf-8"))
    completed = [record for record in records if record.get("status") == "complete"]
    if expected_runs is not None and len(completed) != expected_runs:
        raise ValueError(f"expected {expected_runs} complete runs, found {len(completed)}")
    if len(completed) < 2:
        raise ValueError("at least two complete runs are required for a multi-run split")
    for record in completed:
        path = Path(record["dataset_xlog"])
        if not path.is_file():
            raise FileNotFoundError(path)
        if sha256(path).lower() != str(record["sha256"]).lower():
            raise ValueError(f"{path}: SHA-256 no longer matches the collection record")
    return completed


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--results", type=Path, required=True, help="collection_results.json")
    parser.add_argument("--output", type=Path, required=True, help="output .npz dataset")
    parser.add_argument("--expected-runs", type=int, default=None)
    args = parser.parse_args()
    records = load_verified_runs(args.results, args.expected_runs)
    xlogs = [Path(record["dataset_xlog"]) for record in records]
    manifest = build_dataset(xlogs, args.output)
    manifest["collection_results"] = str(args.results)
    manifest["verified_runs"] = [
        {key: record.get(key) for key in ("id", "dataset_xlog", "sha256", "current_enabled", "current_field")}
        for record in records
    ]
    output_manifest = args.output.with_suffix(args.output.suffix + ".manifest.json")
    output_manifest.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps({"samples": manifest["samples"], "runs": len(records), "output": str(args.output)}, indent=2))


if __name__ == "__main__":
    main()
