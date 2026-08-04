"""Sequential, reproducible Eca A9 SITL data collection for PINN training.

Each manifest entry starts its own UE/SITL run, lets the existing ROS showcase
director drive the unmodified controller, validates that an XLog was emitted,
and copies that XLog into the ignored local learning dataset directory.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[2]
UE_EXE = Path(r"C:\Program Files\Epic Games\UE_5.3\Engine\Binaries\Win64\UnrealEditor.exe")
UE_PROJECT = Path(r"D:\OceanX\engine\OceanX.uproject")
UE_MAP = "/Game/CoreContent/Environments/OceanExampleLevel/OceanExampleLevel"
SITL_EXE = ROOT / "build_sitl" / "Release" / "hydrox_sitl.exe"
XLOG_DIR = UE_EXE.parent / "log"
ROS_ENV = Path(r"C:\Users\Administrator\miniconda3\envs\ros2")
ROS_PYTHON = ROS_ENV / "python.exe"
CONDA_ACTIVATE = Path(r"C:\Users\Administrator\miniconda3\Scripts\activate.bat")
ROS_SETUP = Path(r"D:\OceanX\ros\install\setup.bat")
SHOWCASE_DIRECTOR = Path(r"D:\OceanX\tools\demo_showcase.py")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def resolve(relative: str) -> Path:
    path = Path(relative)
    return path if path.is_absolute() else ROOT / path


def run_director(showcase: Path, scenario: Path, artifact_dir: Path, hold_sec: int,
                 log_path: Path) -> int:
    command = (
        f'call {CONDA_ACTIVATE} {ROS_ENV} && '
        f'call {ROS_SETUP} && '
        f'{ROS_PYTHON} {SHOWCASE_DIRECTOR} '
        f'--showcase {showcase} --scenario {scenario} '
        f'--artifact-dir {artifact_dir} --start-bringup --hold-sec {hold_sec}'
    )
    env = os.environ.copy()
    env["OCEANX_ROOT"] = str(ROOT.parent)
    with log_path.open("w", encoding="utf-8") as stream:
        process = subprocess.Popen(["cmd.exe", "/d", "/c", command], cwd=ROOT, env=env,
                                   stdout=stream, stderr=subprocess.STDOUT)
        try:
            return process.wait(timeout=hold_sec + 150)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                process.kill()
            raise RuntimeError(f"showcase director timed out after {hold_sec + 150} seconds")


def newest_xlog(start_time: float, known: set[Path]) -> Path:
    candidates = [path for path in XLOG_DIR.glob("xlog_ecaa9_train_*.xlog")
                  if path not in known and path.stat().st_mtime >= start_time - 5.0]
    if not candidates:
        raise FileNotFoundError("no new Eca A9 XLog was produced")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def collect(entry: dict[str, Any], scenario: Path, run_root: Path, startup_wait_s: int,
            dry_run: bool) -> dict[str, Any]:
    run_id = str(entry["id"])
    showcase = resolve(str(entry["showcase"]))
    if not showcase.is_file():
        raise FileNotFoundError(showcase)
    current_enabled = bool(entry.get("current_enabled", False))
    current_field = resolve(str(entry["current_field"])) if current_enabled else None
    if current_field is not None and not current_field.is_file():
        raise FileNotFoundError(current_field)
    if not UE_EXE.is_file() or not UE_PROJECT.is_file() or not SITL_EXE.is_file():
        raise FileNotFoundError("UE, project, or hydrox_sitl executable is missing")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    artifacts = run_root / "artifacts" / f"{run_id}_{stamp}"
    artifacts.mkdir(parents=True, exist_ok=False)
    ue_log = artifacts / "unreal.log"
    director_log = artifacts / "director.log"
    ue_args = [
        str(UE_PROJECT), UE_MAP, "-game", "-nullrhi", "-unattended", "-NoSound", "-NoLoadingScreen",
        f"-OceanXScenario={scenario}", f"-OceanXSitlExePath={SITL_EXE}",
        f"-OceanXExitAfterSec={int(entry['ue_exit_sec'])}", f"-OceanXRunId=learning_{run_id}_{stamp}",
        "-OceanXPublishSitlTruthState=true",
        f"-OceanXCurrentFieldEnabled={1 if current_enabled else 0}",
    ]
    if current_field is not None:
        ue_args.append(f"-OceanXCurrentField={current_field}")
    if dry_run:
        return {"id": run_id, "dry_run": True, "ue_command": [str(UE_EXE), *ue_args]}

    known = set(XLOG_DIR.glob("xlog_ecaa9_train_*.xlog"))
    started = time.time()
    print(f"[{run_id}] launching UE", flush=True)
    with ue_log.open("w", encoding="utf-8") as stream:
        ue = subprocess.Popen([str(UE_EXE), *ue_args], cwd=UE_EXE.parent,
                              stdout=stream, stderr=subprocess.STDOUT)
    try:
        time.sleep(startup_wait_s)
        print(f"[{run_id}] starting showcase director", flush=True)
        director_returncode = run_director(showcase, scenario, artifacts, int(entry["hold_sec"]), director_log)
        if director_returncode != 0:
            raise RuntimeError(f"showcase director exited with {director_returncode}")
        try:
            ue.wait(timeout=120)
        except subprocess.TimeoutExpired:
            ue.terminate()
            ue.wait(timeout=30)
        source_xlog = newest_xlog(started, known)
        destination = run_root / "raw" / f"ecaa9_{run_id}_{stamp}.xlog"
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_xlog, destination)
        source_hash = sha256(source_xlog)
        destination_hash = sha256(destination)
        if source_hash != destination_hash:
            raise RuntimeError("XLog SHA-256 mismatch after copy")
        result = {
            "id": run_id, "status": "complete", "showcase": str(showcase),
            "current_enabled": current_enabled, "current_field": str(current_field) if current_field else None,
            "source_xlog": str(source_xlog), "dataset_xlog": str(destination), "sha256": destination_hash,
            "artifacts": str(artifacts), "started_at": datetime.fromtimestamp(started).isoformat(),
            "finished_at": datetime.now().isoformat(),
        }
        print(f"[{run_id}] complete: {destination.name}", flush=True)
        return result
    finally:
        if ue.poll() is None:
            ue.terminate()
            try:
                ue.wait(timeout=30)
            except subprocess.TimeoutExpired:
                ue.kill()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=ROOT / "learning" / "collection" / "ecaa9_multicondition_v1.json")
    parser.add_argument("--run", action="append", dest="run_ids", help="collect only this manifest id; repeatable")
    parser.add_argument("--startup-wait-sec", type=int, default=45)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    scenario = resolve(str(manifest["scenario"]))
    run_root = ROOT / "learning" / "datasets" / "ecaa9_multicondition_v1"
    selected = [entry for entry in manifest["runs"] if not args.run_ids or entry["id"] in args.run_ids]
    if not selected:
        raise ValueError("no manifest runs selected")
    results_path = run_root / "collection_results.json"
    if results_path.is_file():
        results: list[dict[str, Any]] = json.loads(results_path.read_text(encoding="utf-8"))
    else:
        results = []
    for entry in selected:
        result = collect(entry, scenario, run_root, args.startup_wait_sec, args.dry_run)
        results = [prior for prior in results if prior.get("id") != result["id"]]
        results.append(result)
        if not args.dry_run:
            results_path.write_text(json.dumps(results, indent=2), encoding="utf-8")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
