"""PINN-ensemble environment for guarded AUV residual SAC training.

This is a training-only surrogate: it keeps the classical guidance/control
term in every step, then applies the same bounded/rate-limited residual-wrench
rule implemented by ``ResidualSafetyFilter`` in C++.  It has no DDS, actuator,
or C++ control authority.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import gymnasium as gym
import numpy as np
import torch
from gymnasium import spaces

from pinn_residual import ModelConfig, PinnResidualModel
from hydrox_shadow_client import HydroXShadowClient


POLICY_HZ = 20.0
OBSERVATION_NAMES = (
    "u", "v", "w", "p", "q", "r", "depth_error", "surge_error",
    "yaw_rate_error", "sin_yaw_error", "cos_yaw_error", "tau_base_x",
    "tau_base_m", "tau_base_n", "previous_delta_x", "previous_delta_m",
    "previous_delta_n", "pinn_std_x", "pinn_std_m", "pinn_std_n",
)


def wrap_pi(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


@dataclass(frozen=True)
class SafetyLimits:
    """Exact policy-side interpretation of the C++ ResidualSafetyFilter."""

    max_delta: tuple[float, float, float] = (60.0, 8.0, 18.0)
    max_rate: tuple[float, float, float] = (120.0, 16.0, 36.0)
    blend: float = 0.10

    def requested(self, action: np.ndarray, previous: np.ndarray, dt: float) -> np.ndarray:
        requested = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        requested = requested * np.asarray(self.max_delta) * self.blend
        max_step = np.asarray(self.max_rate) * dt
        return previous + np.clip(requested - previous, -max_step, max_step)


@dataclass(frozen=True)
class EnvConfig:
    dt_s: float = 1.0 / POLICY_HZ
    episode_s: float = 30.0
    current_limit_mps: float = 0.25
    reference_n_amplitude_m: float = 85.0
    reference_e_amplitude_m: float = 40.0
    reference_depth_m: float = 100.0
    reference_depth_amplitude_m: float = 6.0
    reference_rate_radps: float = 0.020
    action_effort_weight: float = 0.025
    safety: SafetyLimits = SafetyLimits()
    # Supplying both paths switches the controller+allocator portion of this
    # training surrogate to the real HydroX C++ stack.  The PINN remains a
    # learned dynamics approximation, so this is still Shadow-only training.
    hydrox_shadow_exe: str | None = None
    vehicle_params_path: str | None = None
    vehicle_type: str = "EcaA9"


class PinnEnsemble:
    """Frozen, sampled ensemble dynamics used by the Python-only trainer."""

    def __init__(self, checkpoints: Sequence[str | Path], device: str = "cpu") -> None:
        if not checkpoints:
            raise ValueError("at least one PINN checkpoint is required")
        self.device = torch.device(device)
        self.models = [self._load(Path(path)) for path in checkpoints]

    def _load(self, path: Path) -> PinnResidualModel:
        checkpoint = torch.load(path, map_location=self.device, weights_only=False)
        if checkpoint.get("format") != "hydrox.pinn-residual/v1":
            raise ValueError(f"{path}: unsupported PINN checkpoint")
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
        model.to(self.device).eval()
        return model

    def predict(self, eta: np.ndarray, nu: np.ndarray, tau: np.ndarray,
                member: int) -> tuple[np.ndarray, np.ndarray]:
        """Return one sampled acceleration and ensemble disagreement (both 6-DOF)."""
        eta_t = torch.as_tensor(eta, dtype=torch.float32, device=self.device).unsqueeze(0)
        nu_t = torch.as_tensor(nu, dtype=torch.float32, device=self.device).unsqueeze(0)
        tau_t = torch.as_tensor(tau, dtype=torch.float32, device=self.device).unsqueeze(0)
        with torch.inference_mode():
            accelerations = torch.stack([model(eta_t, nu_t, tau_t)[0][0] for model in self.models])
        acceleration = accelerations[member % len(self.models)].cpu().numpy().astype(np.float64)
        disagreement = accelerations.std(dim=0, correction=0).cpu().numpy().astype(np.float64)
        return acceleration, disagreement


class ResidualAuvEnv(gym.Env[np.ndarray, np.ndarray]):
    """Figure-eight tracking with a classical controller plus safe Δtau action."""

    metadata = {"render_modes": []}

    def __init__(self, ensemble: PinnEnsemble, config: EnvConfig = EnvConfig()) -> None:
        super().__init__()
        self.ensemble = ensemble
        self.config = config
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-20.0, high=20.0,
                                            shape=(len(OBSERVATION_NAMES),), dtype=np.float32)
        self.eta = np.zeros(6, dtype=np.float64)
        self.nu = np.zeros(6, dtype=np.float64)
        self.current_ne = np.zeros(2, dtype=np.float64)
        self.previous_delta = np.zeros(3, dtype=np.float64)
        self.previous_action = np.zeros(3, dtype=np.float64)
        self.member = 0
        self.phase = 0.0
        self.elapsed_s = 0.0
        self.last_base = np.zeros(3, dtype=np.float64)
        self.last_uncertainty = np.zeros(3, dtype=np.float64)
        self.shadow: HydroXShadowClient | None = None
        if (config.hydrox_shadow_exe is None) != (config.vehicle_params_path is None):
            raise ValueError("hydrox_shadow_exe and vehicle_params_path must be supplied together")
        if config.hydrox_shadow_exe is not None:
            self.shadow = HydroXShadowClient(
                config.hydrox_shadow_exe, config.vehicle_params_path,
                vehicle=config.vehicle_type, blend=config.safety.blend,
                max_delta=config.safety.max_delta, max_rate=config.safety.max_rate,
            )

    def _reference(self, lookahead_s: float = 0.0) -> tuple[np.ndarray, float, float, float]:
        theta = self.phase + (self.elapsed_s + lookahead_s) * self.config.reference_rate_radps
        n = self.config.reference_n_amplitude_m * np.sin(theta)
        e = self.config.reference_e_amplitude_m * np.sin(2.0 * theta)
        d = self.config.reference_depth_m + self.config.reference_depth_amplitude_m * np.sin(0.5 * theta)
        n_dot = self.config.reference_n_amplitude_m * self.config.reference_rate_radps * np.cos(theta)
        e_dot = 2.0 * self.config.reference_e_amplitude_m * self.config.reference_rate_radps * np.cos(2.0 * theta)
        heading = float(np.arctan2(e_dot, n_dot))
        surge = float(np.clip(np.hypot(n_dot, e_dot), 0.55, 1.9))
        yaw_rate = float(np.clip(
            (n_dot * (-4.0 * self.config.reference_e_amplitude_m * self.config.reference_rate_radps**2 * np.sin(2.0 * theta))
             - e_dot * (-self.config.reference_n_amplitude_m * self.config.reference_rate_radps**2 * np.sin(theta)))
            / max(n_dot * n_dot + e_dot * e_dot, 1.0e-4), -0.35, 0.35))
        return np.asarray((n, e, d), dtype=np.float64), heading, surge, yaw_rate

    def _controller_contract(self) -> tuple[dict[str, float], np.ndarray]:
        reference, _heading_at_reference, _surge_at_reference, _yaw_rate_at_reference = self._reference()
        lookahead, _unused_heading, surge_ref, yaw_rate_ref = self._reference(lookahead_s=2.5)
        delta_ne = lookahead[:2] - self.eta[:2]
        desired_heading = float(np.arctan2(delta_ne[1], delta_ne[0]))
        heading_error = wrap_pi(desired_heading - self.eta[5])
        horizontal_error = float(np.linalg.norm(reference[:2] - self.eta[:2]))
        depth_error = float(reference[2] - self.eta[2])
        controller = {
            "horizontal_error": horizontal_error,
            "depth_error": depth_error,
            "heading_error": heading_error,
            "surge_error": surge_ref - self.nu[0],
            "yaw_rate_error": yaw_rate_ref - self.nu[5],
        }
        setpoint = np.asarray((
            reference[2], desired_heading, surge_ref, yaw_rate_ref,
            lookahead[0], lookahead[1], reference[2], 1.0,
        ), dtype=np.float64)
        return controller, setpoint

    def _baseline_wrench(self) -> tuple[np.ndarray, dict[str, float]]:
        """Fallback controller used only when no C++ shadow executable was supplied."""
        controller, _setpoint = self._controller_contract()
        heading_error = controller["heading_error"]
        surge_ref = controller["surge_error"] + self.nu[0]
        yaw_rate_ref = controller["yaw_rate_error"] + self.nu[5]
        tau_x = float(np.clip(180.0 * (surge_ref - self.nu[0]), -280.0, 280.0))
        tau_m = float(np.clip(-18.0 * self.eta[4] - 12.0 * self.nu[4], -35.0, 35.0))
        tau_n = float(np.clip(95.0 * heading_error + 38.0 * (yaw_rate_ref - self.nu[5]), -125.0, 125.0))
        return np.asarray((tau_x, tau_m, tau_n), dtype=np.float64), controller

    def _control_step(self, action: np.ndarray, *, reset: bool = False) -> tuple[np.ndarray, dict[str, float], float]:
        """Evaluate one controller tick through C++ when configured, otherwise fallback."""
        controller, setpoint = self._controller_contract()
        if self.shadow is not None:
            result = self.shadow.step(
                reset=reset, dt_s=self.config.dt_s, mode=1, eta=self.eta, nu=self.nu,
                depth_m=float(self.eta[2]), setpoint=setpoint, action=action,
                confidence=1.0, valid=True,
            )
            self.previous_delta = result.applied_delta
            self.last_base = result.base_wrench
            return result.final_wrench, controller, float(np.max(np.abs(result.final_actuator)))
        base, controller = self._baseline_wrench()
        self.previous_delta = self.config.safety.requested(action, self.previous_delta, self.config.dt_s)
        self.last_base = base
        tau = base + self.previous_delta
        saturation = max(abs(tau[0]) / 450.0, abs(tau[1]) / 35.0, abs(tau[2]) / 125.0)
        return tau, controller, saturation

    def _observation(self, controller: dict[str, float]) -> np.ndarray:
        uncertainty = np.clip(self.last_uncertainty / np.asarray((0.6, 0.3, 0.3)), 0.0, 5.0)
        observation = np.concatenate((
            self.nu,
            np.asarray((controller["depth_error"], controller["surge_error"], controller["yaw_rate_error"],
                        np.sin(controller["heading_error"]), np.cos(controller["heading_error"]))),
            self.last_base / np.asarray((280.0, 35.0, 125.0)),
            self.previous_delta / (np.asarray(self.config.safety.max_delta) * self.config.safety.blend),
            uncertainty,
        ))
        return np.clip(observation, self.observation_space.low, self.observation_space.high).astype(np.float32)

    def reset(self, *, seed: int | None = None, options: dict | None = None) -> tuple[np.ndarray, dict]:
        super().reset(seed=seed)
        self.elapsed_s = 0.0
        self.phase = float(self.np_random.uniform(-np.pi, np.pi))
        reference, heading, surge, _yaw_rate = self._reference()
        self.eta[:] = 0.0
        self.eta[:3] = reference + self.np_random.normal(0.0, (2.0, 2.0, 0.7))
        self.eta[4] = float(self.np_random.normal(0.0, 0.03))
        self.eta[5] = wrap_pi(heading + float(self.np_random.normal(0.0, 0.12)))
        self.nu[:] = 0.0
        self.nu[0] = max(0.0, surge + float(self.np_random.normal(0.0, 0.15)))
        self.nu[5] = float(self.np_random.normal(0.0, 0.04))
        self.current_ne = self.np_random.uniform(-self.config.current_limit_mps, self.config.current_limit_mps, size=2)
        self.previous_delta[:] = 0.0
        self.previous_action[:] = 0.0
        self.member = int(self.np_random.integers(len(self.ensemble.models)))
        self.last_uncertainty[:] = 0.0
        if self.shadow is None:
            self.last_base, controller = self._baseline_wrench()
        else:
            # The first policy action must be applied on the first real C++ tick;
            # do not consume a controller update merely to populate reset obs.
            controller, _setpoint = self._controller_contract()
            self.last_base[:] = 0.0
        return self._observation(controller), {"member": self.member}

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, dict]:
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        tau_axes, controller, saturation = self._control_step(action, reset=self.elapsed_s == 0.0)
        tau = np.zeros(6, dtype=np.float64)
        tau[[0, 4, 5]] = tau_axes
        acceleration, disagreement = self.ensemble.predict(self.eta, self.nu, tau, self.member)
        if not np.all(np.isfinite(acceleration)):
            raise RuntimeError("PINN produced non-finite acceleration")
        self.last_uncertainty = disagreement[[0, 4, 5]]
        dt = self.config.dt_s
        self.nu[0] = np.clip(self.nu[0] + acceleration[0] * dt, -0.35, 3.0)
        self.nu[4] = np.clip(self.nu[4] + acceleration[4] * dt, -0.8, 0.8)
        self.nu[5] = np.clip(self.nu[5] + acceleration[5] * dt, -1.3, 1.3)
        self.eta[4] = np.clip(self.eta[4] + self.nu[4] * dt, -0.6, 0.6)
        self.eta[5] = wrap_pi(self.eta[5] + self.nu[5] * dt)
        reference, _heading, _surge, _yaw_rate = self._reference()
        desired_w = np.clip(0.8 * (reference[2] - self.eta[2]) - 0.5 * self.nu[2], -0.7, 0.7)
        self.nu[2] += (desired_w - self.nu[2]) * min(dt / 0.4, 1.0)
        self.eta[2] += self.nu[2] * dt
        self.eta[0] += (np.cos(self.eta[5]) * self.nu[0] + self.current_ne[0]) * dt
        self.eta[1] += (np.sin(self.eta[5]) * self.nu[0] + self.current_ne[1]) * dt
        self.elapsed_s += dt
        next_controller, _setpoint = self._controller_contract()
        track_error = next_controller["horizontal_error"]
        depth_error = abs(next_controller["depth_error"])
        uncertainty_cost = float(np.mean(np.minimum(self.last_uncertainty, 2.0)))
        smoothness = float(np.square(action - self.previous_action).mean())
        action_effort = float(np.square(action).mean())
        self.previous_action = action.copy()
        reward = -(0.16 * min(track_error, 60.0) + 0.08 * depth_error + 0.015 * saturation
                   + 0.010 * smoothness + self.config.action_effort_weight * action_effort
                   + 0.10 * uncertainty_cost)
        terminated = bool(track_error > 75.0 or abs(self.eta[4]) > 0.55 or not np.all(np.isfinite(self.eta)))
        truncated = bool(self.elapsed_s >= self.config.episode_s)
        info = {
            "tracking_error_m": track_error,
            "depth_error_m": depth_error,
            "saturation": saturation,
            "uncertainty": uncertainty_cost,
            "action_effort": action_effort,
            "residual_delta": self.previous_delta.copy(),
            "base_wrench": self.last_base.copy(),
            "control_path": "hydrox_cpp_shadow" if self.shadow is not None else "python_surrogate",
        }
        return self._observation(next_controller), float(reward), terminated, truncated, info

    def close(self) -> None:
        if self.shadow is not None:
            self.shadow.close()
            self.shadow = None
        super().close()


def rollout(env: ResidualAuvEnv, policy, episodes: int, seed: int) -> dict[str, float]:
    """Evaluate a policy callable; returns safety-sensitive episode aggregates."""
    totals: list[float] = []
    tracking: list[float] = []
    terminated: list[float] = []
    for episode in range(episodes):
        observation, _info = env.reset(seed=seed + episode)
        reward_sum = 0.0
        errors: list[float] = []
        done = False
        while not done:
            action = policy(observation)
            observation, reward, terminated_flag, truncated, info = env.step(action)
            reward_sum += reward
            errors.append(float(info["tracking_error_m"]))
            done = terminated_flag or truncated
        totals.append(reward_sum)
        tracking.append(float(np.mean(errors)))
        terminated.append(float(terminated_flag))
    return {
        "mean_return": float(np.mean(totals)),
        "std_return": float(np.std(totals)),
        "mean_tracking_error_m": float(np.mean(tracking)),
        "hard_termination_rate": float(np.mean(terminated)),
    }
