"""Causal low-level residual-RL environment for the ECA A9.

One RL step is 50 ms (20 Hz).  During that interval the original HydroX C++
controller, residual safety filter, and allocator execute five times at
100 Hz.  The resulting normalized actuator channels drive the six-DOF plant
from :mod:`ecaa9_fossen`.

The policy owns only ``[delta_X, delta_N]``.  Pitch/depth remains entirely
classical for the first deployment stage.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Callable

import gymnasium as gym
from gymnasium import spaces
import numpy as np

LEARNING_ROOT = Path(__file__).resolve().parents[1]
HYDROX_ROOT = LEARNING_ROOT.parent
OCEANX_ROOT = HYDROX_ROOT.parent
TRAINING_ROOT = LEARNING_ROOT / "training"
if str(TRAINING_ROOT) not in sys.path:
    # Keep the v2 directory ahead of the legacy training directory.  Both
    # contain files named residual_env.py/train_sac.py; inserting the legacy
    # path at index zero makes independent v2 tools import the wrong module.
    sys.path.append(str(TRAINING_ROOT))

from hydrox_shadow_client import HydroXShadowClient, ShadowResult  # noqa: E402
from ecaa9_fossen import (  # noqa: E402
    DomainSample,
    EcaA9FossenPlant,
    EcaA9Parameters,
    wrap_pi,
)


OBSERVATION_NAMES = (
    "surge_u",
    "sway_v",
    "heave_w",
    "roll_rate_p",
    "pitch_rate_q",
    "yaw_rate_r",
    "sin_pitch",
    "cos_pitch",
    "along_track_error_body",
    "cross_track_error_body",
    "depth_error",
    "surge_error",
    "sin_heading_error",
    "cos_heading_error",
    "yaw_rate_error",
    "current_body_u",
    "current_body_v",
    "base_tau_x",
    "base_tau_m",
    "base_tau_n",
    "previous_delta_x",
    "previous_delta_n",
    "propeller_command",
    "yaw_fin_command",
)

ACTION_NAMES = ("delta_X", "delta_N")


@dataclass(frozen=True)
class ResidualLimits:
    # max_delta is the hard physical proposal limit.  Actual authority is
    # max_delta*blend, exactly as in the C++ ResidualSafetyFilter.
    max_delta_x_n: float = 180.0
    max_delta_m_nm: float = 0.0
    max_delta_n_nm: float = 90.0
    max_rate_x_nps: float = 600.0
    max_rate_m_nmps: float = 0.0
    max_rate_n_nmps: float = 300.0
    blend: float = 0.35

    @property
    def shadow_max_delta(self) -> tuple[float, float, float]:
        return (self.max_delta_x_n, self.max_delta_m_nm, self.max_delta_n_nm)

    @property
    def shadow_max_rate(self) -> tuple[float, float, float]:
        return (self.max_rate_x_nps, self.max_rate_m_nmps, self.max_rate_n_nmps)

    @property
    def effective_x(self) -> float:
        return self.max_delta_x_n * self.blend

    @property
    def effective_n(self) -> float:
        return self.max_delta_n_nm * self.blend


@dataclass(frozen=True)
class EnvConfig:
    policy_hz: float = 20.0
    control_hz: float = 100.0
    episode_s: float = 45.0
    reference_depth_m: float = 100.0
    current_limit_mps: float = 0.35
    observation_noise_scale: float = 0.25
    domain_randomization: bool = True
    limits: ResidualLimits = ResidualLimits()
    shadow_executable: str = str(
        HYDROX_ROOT / "build" / "sitl" / "Release" / "hydrox_control_shadow.exe")
    vehicle_parameters: str = str(
        OCEANX_ROOT / "engine" / "Content" / "Fossen" / "eca_a9_params.json")

    @property
    def control_dt_s(self) -> float:
        return 1.0 / self.control_hz

    @property
    def ticks_per_action(self) -> int:
        ratio = self.control_hz / self.policy_hz
        rounded = int(round(ratio))
        if rounded <= 0 or not math.isclose(ratio, rounded, rel_tol=0.0, abs_tol=1e-9):
            raise ValueError("control_hz must be an integer multiple of policy_hz")
        return rounded


@dataclass
class PathReference:
    north_m: float
    east_m: float
    depth_m: float
    heading_rad: float
    surge_mps: float
    turn_bias_radps: float
    turn_amplitude_radps: float
    turn_frequency_radps: float
    turn_phase_rad: float
    speed_amplitude_mps: float
    depth_amplitude_m: float
    elapsed_s: float = 0.0

    def command(self) -> tuple[float, float, float]:
        yaw_rate = self.turn_bias_radps + self.turn_amplitude_radps * math.sin(
            self.turn_frequency_radps * self.elapsed_s + self.turn_phase_rad)
        surge = self.surge_mps + self.speed_amplitude_mps * math.sin(
            0.47 * self.turn_frequency_radps * self.elapsed_s
            + 0.7 * self.turn_phase_rad)
        depth = self.depth_m + self.depth_amplitude_m * math.sin(
            0.31 * self.turn_frequency_radps * self.elapsed_s
            - 0.4 * self.turn_phase_rad)
        return float(surge), float(yaw_rate), float(depth)

    def advance(self, dt_s: float) -> None:
        surge, yaw_rate, _depth = self.command()
        self.heading_rad = wrap_pi(self.heading_rad + yaw_rate * dt_s)
        self.north_m += surge * math.cos(self.heading_rad) * dt_s
        self.east_m += surge * math.sin(self.heading_rad) * dt_s
        self.elapsed_s += dt_s


class EcaA9LowLevelResidualEnv(gym.Env[np.ndarray, np.ndarray]):
    """HydroX-in-the-loop, two-action residual controller environment."""

    metadata = {"render_modes": []}

    def __init__(self, config: EnvConfig = EnvConfig()) -> None:
        super().__init__()
        self.config = config
        self.parameters = EcaA9Parameters.load(config.vehicle_parameters)
        self.plant = EcaA9FossenPlant(self.parameters)
        self.shadow = HydroXShadowClient(
            config.shadow_executable,
            config.vehicle_parameters,
            vehicle="EcaA9",
            blend=config.limits.blend,
            max_delta=config.limits.shadow_max_delta,
            max_rate=config.limits.shadow_max_rate,
        )
        self.action_space = spaces.Box(-1.0, 1.0, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(
            -6.0, 6.0, shape=(len(OBSERVATION_NAMES),), dtype=np.float32)
        self.reference = PathReference(0.0, 0.0, config.reference_depth_m,
                                       0.0, 1.7, 0.0, 0.1, 0.2, 0.0, 0.2, 2.0)
        self.previous_action = np.zeros(2, dtype=np.float64)
        self.last_shadow: ShadowResult | None = None
        self.last_controller = self._controller_errors()
        self.elapsed_s = 0.0
        self._reset_pending = True
        self._episode_sums: dict[str, float] = {}
        self._episode_samples = 0
        self._observation_bias = np.zeros(4, dtype=np.float64)

    def _sample_domain(self) -> DomainSample:
        if not self.config.domain_randomization:
            return DomainSample()
        magnitude = float(self.np_random.uniform(0.0, self.config.current_limit_mps))
        direction = float(self.np_random.uniform(-math.pi, math.pi))
        return DomainSample(
            rigid_mass_scale=float(self.np_random.uniform(0.92, 1.08)),
            added_mass_scale=float(self.np_random.uniform(0.85, 1.15)),
            damping_scale=float(self.np_random.uniform(0.82, 1.18)),
            thrust_scale=float(self.np_random.uniform(0.88, 1.12)),
            fin_scale=float(self.np_random.uniform(0.85, 1.15)),
            propeller_lag_scale=float(self.np_random.uniform(0.80, 1.25)),
            current_ned_mps=(
                magnitude * math.cos(direction),
                magnitude * math.sin(direction),
                0.0,
            ),
        )

    def _sample_reference(self) -> PathReference:
        return PathReference(
            north_m=0.0,
            east_m=0.0,
            depth_m=self.config.reference_depth_m,
            heading_rad=float(self.np_random.uniform(-math.pi, math.pi)),
            surge_mps=float(self.np_random.uniform(1.35, 2.15)),
            turn_bias_radps=float(self.np_random.uniform(-0.045, 0.045)),
            turn_amplitude_radps=float(self.np_random.uniform(0.06, 0.23)),
            turn_frequency_radps=float(self.np_random.uniform(0.10, 0.28)),
            turn_phase_rad=float(self.np_random.uniform(-math.pi, math.pi)),
            speed_amplitude_mps=float(self.np_random.uniform(0.08, 0.30)),
            depth_amplitude_m=float(self.np_random.uniform(0.0, 3.0)),
        )

    def _controller_errors(self) -> dict[str, float]:
        surge_ref, path_yaw_rate, depth_ref = self.reference.command()
        delta_ned = np.asarray((
            self.reference.north_m - self.plant.eta[0],
            self.reference.east_m - self.plant.eta[1],
            depth_ref - self.plant.eta[2],
        ), dtype=np.float64)
        delta_body = np.asarray((
            math.cos(self.plant.eta[5]) * delta_ned[0]
            + math.sin(self.plant.eta[5]) * delta_ned[1],
            -math.sin(self.plant.eta[5]) * delta_ned[0]
            + math.cos(self.plant.eta[5]) * delta_ned[1],
            delta_ned[2],
        ), dtype=np.float64)
        path_heading_error = wrap_pi(
            self.reference.heading_rad - self.plant.eta[5])
        # External guidance remains deterministic and non-learned.  RL acts
        # below it, at the body-wrench level.
        cross_track_heading = math.atan2(delta_body[1], 7.0)
        yaw_rate_ref = float(np.clip(
            path_yaw_rate + 0.65 * path_heading_error
            + 0.35 * cross_track_heading,
            -0.35, 0.35))
        return {
            "along_track_error_m": float(delta_body[0]),
            "cross_track_error_m": float(delta_body[1]),
            "horizontal_error_m": float(np.linalg.norm(delta_body[:2])),
            "depth_error_m": float(delta_body[2]),
            "heading_error_rad": path_heading_error,
            "surge_ref_mps": surge_ref,
            "surge_error_mps": surge_ref - self.plant.nu[0],
            "yaw_rate_ref_radps": yaw_rate_ref,
            "yaw_rate_error_radps": yaw_rate_ref - self.plant.nu[5],
            "depth_ref_m": depth_ref,
        }

    def _setpoint(self, controller: dict[str, float]) -> np.ndarray:
        return np.asarray((
            controller["depth_ref_m"],
            self.reference.heading_rad,
            controller["surge_ref_mps"],
            controller["yaw_rate_ref_radps"],
            self.reference.north_m,
            self.reference.east_m,
            controller["depth_ref_m"],
            1.0,  # use_yaw_rate_ref
        ), dtype=np.float64)

    def _observation(self, controller: dict[str, float]) -> np.ndarray:
        if self.last_shadow is None:
            base = np.zeros(3, dtype=np.float64)
            delta = np.zeros(3, dtype=np.float64)
            actuator = np.zeros(8, dtype=np.float64)
        else:
            base = self.last_shadow.base_wrench
            delta = self.last_shadow.applied_delta
            actuator = self.last_shadow.final_actuator
        current_body = self.plant.current_body()
        values = np.asarray((
            self.plant.nu[0] / 3.0,
            self.plant.nu[1] / 1.0,
            self.plant.nu[2] / 1.0,
            self.plant.nu[3] / 0.8,
            self.plant.nu[4] / 0.8,
            self.plant.nu[5] / 0.5,
            math.sin(self.plant.eta[4]),
            math.cos(self.plant.eta[4]),
            controller["along_track_error_m"] / 8.0,
            controller["cross_track_error_m"] / 8.0,
            controller["depth_error_m"] / 4.0,
            controller["surge_error_mps"] / 1.0,
            math.sin(controller["heading_error_rad"]),
            math.cos(controller["heading_error_rad"]),
            controller["yaw_rate_error_radps"] / 0.35,
            current_body[0] / max(self.config.current_limit_mps, 0.1),
            current_body[1] / max(self.config.current_limit_mps, 0.1),
            base[0] / 900.0,
            base[1] / 55.0,
            base[2] / 220.0,
            delta[0] / max(self.config.limits.effective_x, 1.0),
            delta[2] / max(self.config.limits.effective_n, 1.0),
            actuator[4],
            0.5 * (actuator[2] - actuator[3]),
        ), dtype=np.float64)
        if self.config.observation_noise_scale > 0.0:
            noise = np.zeros_like(values)
            scale = self.config.observation_noise_scale
            noise[:6] = self.np_random.normal(0.0, 0.008 * scale, size=6)
            noise[8:15] = self.np_random.normal(0.0, 0.005 * scale, size=7)
            values += noise
        return np.clip(values, self.observation_space.low,
                       self.observation_space.high).astype(np.float32)

    def _one_control_tick(self, action: np.ndarray) -> tuple[dict[str, float], float]:
        controller = self._controller_errors()
        residual = np.asarray((action[0], 0.0, action[1]), dtype=np.float64)
        result = self.shadow.step(
            reset=self._reset_pending,
            dt_s=self.config.control_dt_s,
            mode=1,
            eta=self.plant.eta,
            nu=self.plant.nu,
            depth_m=float(self.plant.eta[2]),
            setpoint=self._setpoint(controller),
            action=residual,
            confidence=1.0,
            valid=True,
        )
        self._reset_pending = False
        self.last_shadow = result
        self.plant.step(result.final_actuator, self.config.control_dt_s)
        self.reference.advance(self.config.control_dt_s)
        next_controller = self._controller_errors()
        yaw_fin = 0.5 * (
            result.final_actuator[2] - result.final_actuator[3])
        saturation = float(max(
            abs(result.final_actuator[4]),
            abs(yaw_fin),
            abs(result.final_actuator[0]),
            abs(result.final_actuator[1]),
        ))
        return next_controller, saturation

    def _instant_cost(self, controller: dict[str, float],
                      saturation: float) -> tuple[float, dict[str, float]]:
        components = {
            "cross_track_abs_m": abs(controller["cross_track_error_m"]),
            "along_track_abs_m": abs(controller["along_track_error_m"]),
            "depth_abs_m": abs(controller["depth_error_m"]),
            "surge_abs_mps": abs(controller["surge_error_mps"]),
            "heading_abs_rad": abs(controller["heading_error_rad"]),
            "yaw_rate_abs_radps": abs(controller["yaw_rate_error_radps"]),
            "saturation": saturation,
            "propeller_effort": (
                abs(self.last_shadow.final_actuator[4])
                if self.last_shadow is not None else 0.0),
        }
        cost = (
            0.95 * min((components["cross_track_abs_m"] / 4.0) ** 2, 9.0)
            + 0.20 * min((components["along_track_abs_m"] / 6.0) ** 2, 9.0)
            + 0.35 * min((components["depth_abs_m"] / 2.0) ** 2, 9.0)
            + 0.60 * min((components["surge_abs_mps"] / 0.55) ** 2, 9.0)
            + 0.35 * min((components["heading_abs_rad"] / 0.35) ** 2, 9.0)
            + 0.70 * min((components["yaw_rate_abs_radps"] / 0.18) ** 2, 9.0)
            + 0.20 * max(saturation - 0.88, 0.0) ** 2
        )
        return float(cost), components

    def reset(self, *, seed: int | None = None,
              options: dict | None = None) -> tuple[np.ndarray, dict]:
        super().reset(seed=seed)
        self.reference = self._sample_reference()
        eta = np.zeros(6, dtype=np.float64)
        eta[0:2] = self.np_random.normal(0.0, (1.5, 1.5))
        eta[2] = self.config.reference_depth_m + float(
            self.np_random.normal(0.0, 0.5))
        eta[3] = float(self.np_random.normal(0.0, 0.015))
        eta[4] = float(self.np_random.normal(0.0, 0.025))
        eta[5] = wrap_pi(
            self.reference.heading_rad + float(
                self.np_random.normal(0.0, 0.12)))
        nu = np.zeros(6, dtype=np.float64)
        nu[0] = max(0.4, self.reference.surge_mps + float(
            self.np_random.normal(-0.25, 0.12)))
        nu[5] = float(self.np_random.normal(0.0, 0.025))
        domain = self._sample_domain()
        self.plant.reset(eta, nu, domain)
        self.previous_action[:] = 0.0
        self.last_shadow = None
        self.elapsed_s = 0.0
        self._reset_pending = True
        self._episode_sums = {}
        self._episode_samples = 0
        self.last_controller = self._controller_errors()
        return self._observation(self.last_controller), {
            "domain": domain,
            "action_axes": ACTION_NAMES,
            "control_path": "HydroX C++ GNC+allocator -> OceanX-parity Fossen",
        }

    def step(self, action: np.ndarray):
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        if action.shape != (2,):
            raise ValueError("low-level residual action must be [delta_X, delta_N]")
        previous_horizontal = self.last_controller["horizontal_error_m"]
        accumulated_cost = 0.0
        components: dict[str, float] = {}
        saturation = 0.0
        for _ in range(self.config.ticks_per_action):
            controller, tick_saturation = self._one_control_tick(action)
            tick_cost, tick_components = self._instant_cost(
                controller, tick_saturation)
            accumulated_cost += tick_cost
            saturation = max(saturation, tick_saturation)
            components = tick_components
        self.elapsed_s += 1.0 / self.config.policy_hz
        self.last_controller = controller
        mean_cost = accumulated_cost / self.config.ticks_per_action
        action_effort = float(np.mean(np.square(action)))
        action_smoothness = float(np.mean(np.square(
            action - self.previous_action)))
        progress = float(np.clip(
            previous_horizontal - controller["horizontal_error_m"], -0.5, 0.5))
        reward = (
            -mean_cost
            - 0.018 * action_effort
            - 0.030 * action_smoothness
            + 0.12 * progress
        )
        self.previous_action = action.copy()

        finite = bool(np.isfinite(self.plant.eta).all()
                      and np.isfinite(self.plant.nu).all())
        terminated = bool(
            not finite
            or abs(self.plant.eta[3]) > 0.70
            or abs(self.plant.eta[4]) > 0.60
            or abs(controller["depth_error_m"]) > 15.0
            or controller["horizontal_error_m"] > 30.0
            or np.linalg.norm(self.plant.nu[:3]) > 6.0
        )
        truncated = bool(self.elapsed_s >= self.config.episode_s)

        metrics = {
            **components,
            "horizontal_error_m": controller["horizontal_error_m"],
            "reward_cost": mean_cost,
            "action_effort": action_effort,
            "action_smoothness": action_smoothness,
            "residual_x_n": (
                float(self.last_shadow.applied_delta[0])
                if self.last_shadow is not None else 0.0),
            "residual_n_nm": (
                float(self.last_shadow.applied_delta[2])
                if self.last_shadow is not None else 0.0),
        }
        for name, value in metrics.items():
            self._episode_sums[name] = self._episode_sums.get(name, 0.0) + float(value)
        self._episode_samples += 1
        info = {
            **metrics,
            "elapsed_s": self.elapsed_s,
            "hard_termination": terminated,
            "state_eta": self.plant.eta.copy(),
            "state_nu": self.plant.nu.copy(),
            "reference_ne": np.asarray(
                (self.reference.north_m, self.reference.east_m)),
        }
        if terminated or truncated:
            info["episode_metrics"] = {
                name: total / max(self._episode_samples, 1)
                for name, total in self._episode_sums.items()
            }
            info["episode_metrics"]["hard_termination"] = float(terminated)
        return (self._observation(controller), float(reward), terminated,
                truncated, info)

    def close(self) -> None:
        if getattr(self, "shadow", None) is not None:
            self.shadow.close()
            self.shadow = None
        super().close()


def evaluate_policy(
    env_factory: Callable[[], EcaA9LowLevelResidualEnv],
    policy: Callable[[np.ndarray], np.ndarray],
    seeds: list[int],
) -> tuple[dict[str, float], list[dict[str, np.ndarray]]]:
    """Evaluate fixed seeds and retain trajectories for comparison plots."""
    episodes: list[dict[str, float]] = []
    trajectories: list[dict[str, np.ndarray]] = []
    env = env_factory()
    try:
        for seed in seeds:
            observation, _ = env.reset(seed=seed)
            series: dict[str, list] = {
                "time_s": [], "north_m": [], "east_m": [],
                "reference_north_m": [], "reference_east_m": [],
                "cross_track_error_m": [], "surge_error_mps": [],
                "yaw_rate_error_radps": [], "depth_error_m": [],
                "residual_x_n": [], "residual_n_nm": [],
            }
            done = False
            final_info: dict = {}
            while not done:
                action = np.asarray(policy(observation), dtype=np.float32)
                observation, _reward, terminated, truncated, info = env.step(action)
                series["time_s"].append(info["elapsed_s"])
                series["north_m"].append(info["state_eta"][0])
                series["east_m"].append(info["state_eta"][1])
                series["reference_north_m"].append(info["reference_ne"][0])
                series["reference_east_m"].append(info["reference_ne"][1])
                series["cross_track_error_m"].append(info["cross_track_abs_m"])
                series["surge_error_mps"].append(info["surge_abs_mps"])
                series["yaw_rate_error_radps"].append(info["yaw_rate_abs_radps"])
                series["depth_error_m"].append(info["depth_abs_m"])
                series["residual_x_n"].append(info["residual_x_n"])
                series["residual_n_nm"].append(info["residual_n_nm"])
                done = terminated or truncated
                final_info = info
            episodes.append(dict(final_info["episode_metrics"]))
            trajectories.append({
                name: np.asarray(values, dtype=np.float64)
                for name, values in series.items()
            })
    finally:
        env.close()
    names = sorted({name for episode in episodes for name in episode})
    summary = {
        name: float(np.mean([episode[name] for episode in episodes]))
        for name in names
    }
    return summary, trajectories
