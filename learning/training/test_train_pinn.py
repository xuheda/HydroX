"""End-to-end smoke test for the PINN trainer with a synthetic local dataset."""
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np


def main() -> None:
    rng = np.random.default_rng(7)
    samples = 32
    eta = rng.normal(size=(samples, 6)).astype(np.float32) * 0.1
    nu = rng.normal(size=(samples, 6)).astype(np.float32) * 0.1
    tau = rng.normal(size=(samples, 6)).astype(np.float32)
    dt = np.full(samples, 0.01, dtype=np.float32)
    next_nu = nu + tau / 70.0 * dt[:, None]
    with tempfile.TemporaryDirectory() as directory:
        directory = Path(directory)
        dataset = directory / "fixture.npz"
        np.savez_compressed(dataset, observation=np.zeros((samples, 30), dtype=np.float32), eta=eta,
                            nu=nu, next_nu=next_nu, tau_base=tau,
                            actuator=np.zeros((samples, 8), dtype=np.float32), dt=dt,
                            source_run=np.repeat(np.asarray((0, 1), dtype=np.int32), samples // 2),
                            feature_names=np.asarray(["fixture"]))
        output = directory / "result"
        trainer = Path(__file__).with_name("train_pinn.py")
        subprocess.run([sys.executable, str(trainer), str(dataset), "--output", str(output),
                        "--epochs", "1", "--ensemble", "1", "--batch-size", "8", "--device", "cpu"],
                       check=True)
        report = json.loads((output / "pinn_metrics.json").read_text(encoding="utf-8"))
        if len(report["members"]) != 1:
            raise AssertionError("trainer did not write the ensemble report")
    print("test_train_pinn: PASS")


if __name__ == "__main__":
    main()
