"""Persistent client for the offline C++ HydroX control-shadow executable.

The process only receives synthetic or recorded state/setpoint rows over stdin
and writes replay results to stdout.  It has no network, DDS, UE, or actuator
interfaces.  Keeping the process persistent lets the C++ controller preserve
the same internal PID/SMC/filter state across an RL episode.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import subprocess
from typing import Sequence

import numpy as np


INPUT_HEADER = (
    "reset,dt_s,mode,eta_n,eta_e,eta_d,eta_roll,eta_pitch,eta_yaw,"
    "nu_u,nu_v,nu_w,nu_p,nu_q,nu_r,depth_m,sp_depth,sp_heading,sp_surge,"
    "sp_use_yaw_rate,sp_yaw_rate,sp_wp_n,sp_wp_e,sp_wp_d,action_x,action_m,"
    "action_n,confidence,valid"
)
OUTPUT_HEADER = (
    "index,base_x,base_m,base_n,final_x,final_m,final_n,delta_x,delta_m,delta_n,"
    "base_ch0,base_ch1,base_ch2,base_ch3,base_ch4,base_ch5,base_ch6,base_ch7,base_rpm,"
    "final_ch0,final_ch1,final_ch2,final_ch3,final_ch4,final_ch5,final_ch6,final_ch7,final_rpm"
)


@dataclass(frozen=True)
class ShadowResult:
    base_wrench: np.ndarray
    final_wrench: np.ndarray
    applied_delta: np.ndarray
    base_actuator: np.ndarray
    final_actuator: np.ndarray
    base_rpm: float
    final_rpm: float


class HydroXShadowClient:
    """One C++ control stack, scoped to one Python environment/episode stream."""

    def __init__(self, executable: str | Path, vehicle_params: str | Path,
                 *, vehicle: str = "EcaA9", blend: float = 0.10,
                 max_delta: Sequence[float] = (60.0, 8.0, 18.0),
                 max_rate: Sequence[float] = (120.0, 16.0, 36.0),
                 min_confidence: float = 0.75) -> None:
        self.executable = Path(executable)
        self.vehicle_params = Path(vehicle_params)
        if not self.executable.is_file():
            raise FileNotFoundError(f"HydroX control-shadow executable not found: {self.executable}")
        if not self.vehicle_params.is_file():
            raise FileNotFoundError(f"HydroX vehicle parameter file not found: {self.vehicle_params}")
        if len(max_delta) != 3 or len(max_rate) != 3:
            raise ValueError("shadow residual limits must be X,M,N triplets")
        command = [
            str(self.executable), "--stdio", "--vehicle", vehicle,
            "--vehicle-params", str(self.vehicle_params), "--blend", f"{blend:.17g}",
            "--min-confidence", f"{min_confidence:.17g}",
            "--max-delta", ",".join(f"{value:.17g}" for value in max_delta),
            "--max-rate", ",".join(f"{value:.17g}" for value in max_rate),
        ]
        self.process = subprocess.Popen(
            command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, encoding="utf-8", bufsize=1,
        )
        assert self.process.stdin is not None and self.process.stdout is not None
        self.process.stdin.write(INPUT_HEADER + "\n")
        self.process.stdin.flush()
        actual_header = self.process.stdout.readline().strip()
        if actual_header != OUTPUT_HEADER:
            self.close()
            raise RuntimeError(
                "HydroX control-shadow did not accept the replay contract: "
                f"{self._stderr_text() or actual_header!r}"
            )

    def _stderr_text(self) -> str:
        if self.process.stderr is None:
            return ""
        try:
            return self.process.stderr.read().strip()
        except OSError:
            return ""

    def _check_running(self) -> None:
        code = self.process.poll()
        if code is not None:
            raise RuntimeError(f"HydroX control-shadow exited with {code}: {self._stderr_text()}")

    def step(self, *, reset: bool, dt_s: float, mode: int, eta: np.ndarray, nu: np.ndarray,
             depth_m: float, setpoint: Sequence[float], action: np.ndarray,
             confidence: float = 1.0, valid: bool = True) -> ShadowResult:
        self._check_running()
        eta = np.asarray(eta, dtype=np.float64)
        nu = np.asarray(nu, dtype=np.float64)
        action = np.asarray(action, dtype=np.float64)
        if eta.shape != (6,) or nu.shape != (6,) or action.shape != (3,) or len(setpoint) != 8:
            raise ValueError("invalid HydroX control-shadow state, action, or setpoint shape")
        if not (np.isfinite(eta).all() and np.isfinite(nu).all() and np.isfinite(action).all() and
                np.isfinite(np.asarray(setpoint, dtype=np.float64)).all() and np.isfinite(depth_m) and
                np.isfinite(dt_s) and dt_s > 0.0):
            raise ValueError("HydroX control-shadow inputs must be finite and have positive dt")
        row = [
            float(reset), dt_s, int(mode), *eta.tolist(), *nu.tolist(), depth_m,
            float(setpoint[0]), float(setpoint[1]), float(setpoint[2]), float(bool(setpoint[7])),
            float(setpoint[3]), float(setpoint[4]), float(setpoint[5]), float(setpoint[6]),
            *action.tolist(), confidence, float(valid),
        ]
        assert self.process.stdin is not None and self.process.stdout is not None
        self.process.stdin.write(",".join(f"{float(value):.17g}" for value in row) + "\n")
        self.process.stdin.flush()
        response = self.process.stdout.readline().strip()
        if not response:
            self._check_running()
            raise RuntimeError("HydroX control-shadow returned an empty response")
        values = np.fromstring(response, dtype=np.float64, sep=",")
        if values.shape != (28,) or not np.isfinite(values).all():
            raise RuntimeError(f"invalid HydroX control-shadow response: {response!r}")
        return ShadowResult(
            base_wrench=np.asarray((values[1], values[2], values[3]), dtype=np.float64),
            final_wrench=np.asarray((values[4], values[5], values[6]), dtype=np.float64),
            applied_delta=np.asarray((values[7], values[8], values[9]), dtype=np.float64),
            base_actuator=values[10:18].copy(), final_actuator=values[19:27].copy(),
            base_rpm=float(values[18]), final_rpm=float(values[27]),
        )

    def close(self) -> None:
        process = getattr(self, "process", None)
        if process is None:
            return
        if process.stdin is not None and not process.stdin.closed:
            process.stdin.close()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2.0)
        # Explicitly release the two read handles.  Relying on CPython
        # finalization emits ResourceWarning during vector-environment churn
        # and can exhaust Windows pipe handles in long multi-process runs.
        if process.stdout is not None and not process.stdout.closed:
            process.stdout.close()
        if process.stderr is not None and not process.stderr.closed:
            process.stderr.close()
        self.process = None
