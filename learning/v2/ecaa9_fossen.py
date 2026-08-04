"""OceanX-parity ECA A9 Fossen plant used by low-level residual RL v2.

The previous pilot environment advanced position with hand-written kinematics
after predicting only X/M/N acceleration.  That made the residual action only
partly causal.  This module instead mirrors the authoritative OceanX path:

* parameters come from ``engine/Content/Fossen/eca_a9_params.json``;
* HydroX allocator channels are reconstructed into the same body wrench;
* propeller shaft lag and the open-water KT/KQ law are retained;
* the complete 6-DOF relative-current equation is integrated at 100 Hz.

The vehicle is normally trained well below the surface, so the UE surface-wave
and breach guards are deliberately outside this local trainer.  Final
acceptance still takes place in OceanX SITL.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np


def wrap_pi(angle: float) -> float:
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.asarray(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)),
                      dtype=np.float64)


def _rotation_body_to_ned(eta: np.ndarray) -> np.ndarray:
    phi, theta, psi = eta[3:6]
    cphi, sphi = math.cos(phi), math.sin(phi)
    ctheta, stheta = math.cos(theta), math.sin(theta)
    cpsi, spsi = math.cos(psi), math.sin(psi)
    return np.asarray((
        (cpsi * ctheta, -spsi * cphi + cpsi * stheta * sphi,
         spsi * sphi + cpsi * cphi * stheta),
        (spsi * ctheta, cpsi * cphi + sphi * stheta * spsi,
         -cpsi * sphi + stheta * spsi * cphi),
        (-stheta, ctheta * sphi, ctheta * cphi),
    ), dtype=np.float64)


def _attitude_rate(eta: np.ndarray, nu: np.ndarray) -> np.ndarray:
    phi, theta = eta[3], eta[4]
    cphi, sphi = math.cos(phi), math.sin(phi)
    ctheta = math.cos(theta)
    if abs(ctheta) < 1.0e-6:
        ctheta = math.copysign(1.0e-6, ctheta if ctheta else 1.0)
    ttheta = math.sin(theta) / ctheta
    transform = np.asarray((
        (1.0, sphi * ttheta, cphi * ttheta),
        (0.0, cphi, -sphi),
        (0.0, sphi / ctheta, cphi / ctheta),
    ), dtype=np.float64)
    return np.concatenate((_rotation_body_to_ned(eta) @ nu[:3],
                           transform @ nu[3:6]))


def _coriolis(mass: np.ndarray, nu: np.ndarray) -> np.ndarray:
    """Match ``FossenMath::M2C`` for a symmetric six-axis mass matrix."""
    symmetric = 0.5 * (mass + mass.T)
    momentum_linear = symmetric[:3, :3] @ nu[:3] + symmetric[:3, 3:] @ nu[3:]
    momentum_angular = symmetric[3:, :3] @ nu[:3] + symmetric[3:, 3:] @ nu[3:]
    result = np.zeros((6, 6), dtype=np.float64)
    result[:3, 3:] = -_skew(momentum_linear)
    result[3:, :3] = -_skew(momentum_linear)
    result[3:, 3:] = -_skew(momentum_angular)
    return result


def _rigid_body_mass(mass: float, inertia: np.ndarray,
                     center_of_gravity: np.ndarray) -> np.ndarray:
    # This is the same homogeneous-transform construction used by the OceanX
    # parameter loader.  ECA A9 currently has r_bg=0, but retaining the general
    # form keeps payload-offset experiments honest.
    transform = np.eye(6, dtype=np.float64)
    transform[:3, 3:] = _skew(center_of_gravity).T
    diagonal = np.zeros((6, 6), dtype=np.float64)
    diagonal[:3, :3] = np.eye(3) * mass
    diagonal[3:, 3:] = inertia
    return transform.T @ diagonal @ transform


@dataclass(frozen=True)
class DomainSample:
    """Per-episode sim-to-real randomisation.

    Values stay intentionally narrow.  OceanX SITL remains the authority; this
    spread exists to prevent the policy from exploiting one exact parameter
    tuple.
    """

    rigid_mass_scale: float = 1.0
    added_mass_scale: float = 1.0
    damping_scale: float = 1.0
    thrust_scale: float = 1.0
    fin_scale: float = 1.0
    propeller_lag_scale: float = 1.0
    current_ned_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class EcaA9Parameters:
    rho: float
    mass: float
    volume_m3: float
    inertia: np.ndarray
    r_bg: np.ndarray
    r_bb: np.ndarray
    added_mass: np.ndarray
    source_forward_speed_damping: np.ndarray
    length_m: float
    diameter_m: float
    fin_area_m2: float
    allocator_cl_s: float
    allocator_cl_r: float
    allocator_u_min_mps: float
    fin_cop: np.ndarray
    delta_max_rad: float
    max_thrust_n: float
    min_thrust_n: float
    propeller_time_constant_s: float
    max_rpm: float
    propeller_diameter_m: float
    thrust_deduction: float
    kt_0: float
    kq_0: float
    kt_max: float
    kq_max: float
    wake_fraction: float
    max_advance_ratio: float

    @staticmethod
    def load(path: str | Path) -> "EcaA9Parameters":
        source = Path(path)
        raw = json.loads(source.read_text(encoding="utf-8"))
        geometry = raw["geometry"]
        inertia = raw["inertia"]
        hydro = raw["hydrodynamics"]
        fins = raw["actuators"]["fins"]
        propeller = raw["actuators"]["propeller"]
        inertia_tensor = np.asarray((
            (float(inertia["Ixx"]), -float(inertia.get("Ixy", 0.0)),
             -float(inertia.get("Ixz", 0.0))),
            (-float(inertia.get("Ixy", 0.0)), float(inertia["Iyy"]),
             -float(inertia.get("Iyz", 0.0))),
            (-float(inertia.get("Ixz", 0.0)), -float(inertia.get("Iyz", 0.0)),
             float(inertia["Izz"])),
        ), dtype=np.float64)
        return EcaA9Parameters(
            rho=float(inertia["rho"]),
            mass=float(inertia["mass"]),
            volume_m3=float(inertia["volume_m3"]),
            inertia=inertia_tensor,
            r_bg=np.asarray(inertia["r_bg"], dtype=np.float64),
            r_bb=np.asarray(inertia["r_bb"], dtype=np.float64),
            added_mass=np.asarray(hydro["added_mass"], dtype=np.float64),
            source_forward_speed_damping=np.asarray(
                hydro["linear_damping_forward_speed"], dtype=np.float64),
            length_m=float(geometry["length"]),
            diameter_m=float(geometry["diameter"]),
            fin_area_m2=float(fins["fin_area_m2"]),
            allocator_cl_s=float(fins.get("allocator_CL_s", fins["CL_delta_s"])),
            allocator_cl_r=float(fins.get("allocator_CL_r", fins["CL_delta_r"])),
            allocator_u_min_mps=float(
                fins.get("allocator_u_min", fins.get("u_min", 0.3))),
            fin_cop=np.asarray(fins["fin_cop"], dtype=np.float64),
            delta_max_rad=math.radians(float(fins["fin_max_deflection_deg"])),
            max_thrust_n=float(propeller["max_thrust_N"]),
            min_thrust_n=float(propeller["min_thrust_N"]),
            propeller_time_constant_s=float(propeller["T_n"]),
            max_rpm=float(propeller["n_max_rpm"]),
            propeller_diameter_m=float(propeller["D_prop"]),
            thrust_deduction=float(propeller["t_prop"]),
            kt_0=float(propeller["KT_0"]),
            kq_0=float(propeller["KQ_0"]),
            kt_max=float(propeller["KT_max"]),
            kq_max=float(propeller["KQ_max"]),
            wake_fraction=float(propeller["w_wake"]),
            max_advance_ratio=float(propeller["Ja_max"]),
        )


class EcaA9FossenPlant:
    """Stateful 6-DOF plant accepting HydroX normalized allocator channels."""

    def __init__(self, parameters: EcaA9Parameters) -> None:
        self.parameters = parameters
        self.eta = np.zeros(6, dtype=np.float64)
        self.nu = np.zeros(6, dtype=np.float64)
        self.shaft_rpm = 0.0
        self.domain = DomainSample()
        self.mrb = np.eye(6, dtype=np.float64)
        self.ma = np.eye(6, dtype=np.float64)
        self.mass = np.eye(6, dtype=np.float64)
        self.mass_inverse = np.eye(6, dtype=np.float64)
        self.last_applied_wrench = np.zeros(6, dtype=np.float64)
        self.configure_domain(self.domain)

    def configure_domain(self, domain: DomainSample) -> None:
        self.domain = domain
        p = self.parameters
        self.mrb = _rigid_body_mass(
            p.mass * domain.rigid_mass_scale,
            p.inertia * domain.rigid_mass_scale,
            p.r_bg,
        )
        self.ma = p.added_mass * domain.added_mass_scale
        self.mass = self.mrb + self.ma
        self.mass_inverse = np.linalg.inv(self.mass)

    def reset(self, eta: np.ndarray, nu: np.ndarray,
              domain: DomainSample = DomainSample()) -> None:
        eta = np.asarray(eta, dtype=np.float64)
        nu = np.asarray(nu, dtype=np.float64)
        if eta.shape != (6,) or nu.shape != (6,):
            raise ValueError("ECA A9 pose and velocity must both be six-vectors")
        self.configure_domain(domain)
        self.eta = eta.copy()
        self.nu = nu.copy()
        self.shaft_rpm = 0.0
        self.last_applied_wrench[:] = 0.0

    def current_body(self) -> np.ndarray:
        current_ned = np.asarray(self.domain.current_ned_mps, dtype=np.float64)
        return _rotation_body_to_ned(self.eta).T @ current_ned

    def _target_rpm(self, thrust_fraction: float) -> float:
        p = self.parameters
        target_thrust = float(np.clip(thrust_fraction, -1.0, 1.0)) * p.max_thrust_n
        coefficient = p.kt_0 * p.rho * p.propeller_diameter_m**4
        if abs(target_thrust) < 1.0e-4 or coefficient < 1.0e-10:
            return 0.0
        target_rps = math.copysign(math.sqrt(abs(target_thrust) / coefficient),
                                   target_thrust)
        return float(np.clip(target_rps * 60.0, -p.max_rpm, p.max_rpm))

    def _propeller_wrench(self) -> tuple[float, float]:
        p = self.parameters
        speed = float(np.linalg.norm(self.nu[:3]))
        revolutions_per_second = self.shaft_rpm / 60.0
        absolute_n = abs(revolutions_per_second)
        advance_speed = (1.0 - p.wake_fraction) * speed
        diameter4 = p.propeller_diameter_m**4
        if revolutions_per_second >= 0.0:
            thrust = p.rho * diameter4 * (
                p.kt_0 * absolute_n * revolutions_per_second
                + (p.kt_max - p.kt_0) / max(p.max_advance_ratio, 1.0e-9)
                * (advance_speed / p.propeller_diameter_m) * absolute_n
            )
            torque = p.rho * diameter4 * p.propeller_diameter_m * (
                p.kq_0 * absolute_n * revolutions_per_second
                + (p.kq_max - p.kq_0) / max(p.max_advance_ratio, 1.0e-9)
                * (advance_speed / p.propeller_diameter_m) * absolute_n
            )
        else:
            thrust = (p.rho * diameter4 * p.kt_0 * absolute_n
                      * revolutions_per_second)
            torque = (p.rho * diameter4 * p.propeller_diameter_m * p.kq_0
                      * absolute_n * revolutions_per_second)
        thrust = float(np.clip(thrust, p.min_thrust_n, p.max_thrust_n))
        return thrust, torque

    def reconstruct_allocator_wrench(self, controls: np.ndarray,
                                     dt_s: float) -> np.ndarray:
        """Mirror ``FossenActuators::ReconstructAllocatorTau``."""
        p = self.parameters
        channels = np.zeros(8, dtype=np.float64)
        supplied = np.asarray(controls, dtype=np.float64).reshape(-1)
        channels[:min(8, supplied.size)] = supplied[:8]
        channels = np.clip(channels, -1.0, 1.0)

        target_rpm = self._target_rpm(channels[4])
        time_constant = max(
            p.propeller_time_constant_s * self.domain.propeller_lag_scale, 1.0e-3)
        alpha = 1.0 - math.exp(-dt_s / time_constant)
        self.shaft_rpm += (target_rpm - self.shaft_rpm) * alpha

        delta_ss, delta_ps, delta_br, delta_tr = (
            channels[:4] * p.delta_max_rad)
        effective_speed = max(abs(self.nu[0]), p.allocator_u_min_mps, 1.0e-3)
        dynamic_pressure = 0.5 * p.rho * effective_speed**2
        x_fin = max(float(np.max(np.abs(p.fin_cop[:, 0]))), p.length_m * 0.5)
        denom_s = 2.0 * dynamic_pressure * p.fin_area_m2 * p.allocator_cl_s
        denom_r = dynamic_pressure * p.fin_area_m2 * p.allocator_cl_r * x_fin
        thrust, _torque = self._propeller_wrench()

        wrench = np.zeros(6, dtype=np.float64)
        wrench[0] = ((1.0 - p.thrust_deduction) * thrust
                     * self.domain.thrust_scale)
        wrench[2] = ((delta_ss + delta_ps) * denom_s
                     * self.domain.fin_scale)
        wrench[4] = ((delta_ss - delta_ps) * denom_s * x_fin
                     * self.domain.fin_scale)
        wrench[5] = ((delta_br - delta_tr) * denom_r
                     * self.domain.fin_scale)
        return wrench

    def _restoring_forces(self) -> np.ndarray:
        p = self.parameters
        phi, theta = self.eta[3], self.eta[4]
        sphi, cphi = math.sin(phi), math.cos(phi)
        stheta, ctheta = math.sin(theta), math.cos(theta)
        weight = p.mass * self.domain.rigid_mass_scale * 9.81
        buoyancy = p.rho * p.volume_m3 * 9.81
        xg_w, yg_w, zg_w = p.r_bg * weight
        xb_b, yb_b, zb_b = p.r_bb * buoyancy
        return np.asarray((
            (weight - buoyancy) * stheta,
            -(weight - buoyancy) * ctheta * sphi,
            -(weight - buoyancy) * ctheta * cphi,
            -(yg_w - yb_b) * ctheta * cphi + (zg_w - zb_b) * ctheta * sphi,
            (zg_w - zb_b) * stheta + (xg_w - xb_b) * ctheta * cphi,
            -(xg_w - xb_b) * ctheta * sphi - (yg_w - yb_b) * stheta,
        ), dtype=np.float64)

    def step(self, controls: np.ndarray, dt_s: float) -> np.ndarray:
        if not np.isfinite(dt_s) or dt_s <= 0.0:
            raise ValueError("plant dt must be finite and positive")
        applied = self.reconstruct_allocator_wrench(controls, dt_s)
        current_body_linear = self.current_body()
        current_body = np.concatenate((current_body_linear, np.zeros(3)))
        relative_velocity = self.nu - current_body
        # Uniform NED current has zero inertial derivative.  Its derivative in
        # the rotating body frame is -omega x current_body.
        current_derivative = np.concatenate((
            -np.cross(self.nu[3:6], current_body_linear), np.zeros(3)))

        damping = (self.parameters.source_forward_speed_damping
                   @ relative_velocity) * relative_velocity[0]
        hydro = (damping * self.domain.damping_scale
                 - self._restoring_forces())
        rigid_coriolis = _coriolis(self.mrb, self.nu) @ self.nu
        added_coriolis = _coriolis(self.ma, relative_velocity) @ relative_velocity
        net = (hydro + applied + self.ma @ current_derivative
               - rigid_coriolis - added_coriolis)
        acceleration = self.mass_inverse @ np.clip(
            net, (-50000.0, -50000.0, -50000.0, -20000.0, -20000.0, -20000.0),
            (50000.0, 50000.0, 50000.0, 20000.0, 20000.0, 20000.0))
        self.nu += acceleration * dt_s
        self.eta += _attitude_rate(self.eta, self.nu) * dt_s
        self.eta[3] = wrap_pi(self.eta[3])
        self.eta[4] = float(np.clip(self.eta[4],
                                    -0.5 * math.pi + 1.0e-3,
                                    0.5 * math.pi - 1.0e-3))
        self.eta[5] = wrap_pi(self.eta[5])
        self.last_applied_wrench = applied
        return acceleration
