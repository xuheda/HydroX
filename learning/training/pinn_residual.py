"""Grey-box Fossen-prior residual dynamics model used by the RL environment."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import torch
from torch import Tensor, nn
from torch.nn import functional as F


AXIS_NAMES = ("X", "Y", "Z", "K", "M", "N")


def axis_mask(names: Iterable[str]) -> Tensor:
    selected = {name.upper() for name in names}
    unknown = selected.difference(AXIS_NAMES)
    if unknown:
        raise ValueError(f"unknown wrench axes: {sorted(unknown)}")
    return torch.tensor([1.0 if name in selected else 0.0 for name in AXIS_NAMES])


def dynamics_input(eta: Tensor, nu: Tensor, tau: Tensor) -> Tensor:
    """19 features with periodic attitude encoding, in the HydroX NED/FRD convention."""
    angles = eta[..., 3:6]
    return torch.cat((nu, eta[..., 2:3], torch.sin(angles), torch.cos(angles), tau), dim=-1)


class DiagonalFossenPrior(nn.Module):
    """Deliberately conservative diagonal prior; the residual handles remaining dynamics."""

    def __init__(self, mass_diag: Tensor) -> None:
        super().__init__()
        if mass_diag.shape != (6,) or torch.any(mass_diag <= 0):
            raise ValueError("mass_diag must contain six positive entries")
        # Store an unconstrained parameter so the effective inertia stays positive.
        self.raw_mass = nn.Parameter(torch.log(torch.expm1(mass_diag)))
        self.raw_linear_damping = nn.Parameter(torch.full((6,), -2.0))
        self.raw_quadratic_damping = nn.Parameter(torch.full((6,), -4.0))

    @property
    def mass(self) -> Tensor:
        return F.softplus(self.raw_mass) + 1.0e-3

    def forward(self, nu: Tensor, tau: Tensor, residual_force: Tensor) -> Tensor:
        linear = F.softplus(self.raw_linear_damping)
        quadratic = F.softplus(self.raw_quadratic_damping)
        damping = linear * nu + quadratic * nu.abs() * nu
        return (tau - damping + residual_force) / self.mass


class PinnResidualModel(nn.Module):
    """Fossen acceleration prior plus a bounded, uncertainty-aware force residual."""

    def __init__(self, input_mean: Tensor, input_std: Tensor, mass_diag: Tensor,
                 enabled_axes: Tensor, residual_force_limit: Tensor,
                 width: int = 128, depth: int = 3) -> None:
        super().__init__()
        if input_mean.shape != (19,) or input_std.shape != (19,):
            raise ValueError("expected normalization statistics for 19 dynamics features")
        self.register_buffer("input_mean", input_mean.clone())
        self.register_buffer("input_std", input_std.clamp_min(1.0e-5).clone())
        self.register_buffer("enabled_axes", enabled_axes.clone())
        self.register_buffer("residual_force_limit", residual_force_limit.clone())
        layers: list[nn.Module] = []
        current = 19
        for _ in range(depth):
            layers.extend((nn.Linear(current, width), nn.SiLU()))
            current = width
        layers.append(nn.Linear(current, 12))  # force residual mean and diagonal log variance
        self.network = nn.Sequential(*layers)
        self.prior = DiagonalFossenPrior(mass_diag)

    def forward(self, eta: Tensor, nu: Tensor, tau: Tensor) -> tuple[Tensor, Tensor, Tensor]:
        raw_input = dynamics_input(eta, nu, tau)
        normalized = (raw_input - self.input_mean) / self.input_std
        output = self.network(normalized)
        residual_force = torch.tanh(output[..., :6]) * self.residual_force_limit * self.enabled_axes
        log_variance = output[..., 6:].clamp(-9.0, 5.0)
        acceleration = self.prior(nu, tau, residual_force)
        return acceleration, residual_force, log_variance

    def loss(self, eta: Tensor, nu: Tensor, tau: Tensor, target_acceleration: Tensor,
             dissipation_power_allowance: float = 0.0) -> dict[str, Tensor]:
        predicted, residual_force, log_variance = self(eta, nu, tau)
        active = self.enabled_axes
        error_sq = (predicted - target_acceleration).square()
        variance = torch.exp(log_variance)
        nll = 0.5 * (error_sq / variance + log_variance)
        data_loss = (nll * active).sum(dim=-1).mean() / active.sum().clamp_min(1.0)
        # A hydrodynamic residual should not inject unlimited energy into the vehicle.
        injected_power = (nu * residual_force).sum(dim=-1)
        dissipation_loss = F.relu(injected_power - dissipation_power_allowance).square().mean()
        zero_eta = torch.zeros(1, 6, device=eta.device, dtype=eta.dtype)
        zero_nu = torch.zeros_like(zero_eta)
        zero_tau = torch.zeros_like(zero_eta)
        _, rest_force, _ = self(zero_eta, zero_nu, zero_tau)
        rest_loss = rest_force.square().mean()
        return {"data": data_loss, "dissipation": dissipation_loss, "rest": rest_loss,
                "total": data_loss + 1.0e-4 * dissipation_loss + 1.0e-3 * rest_loss}


@dataclass(frozen=True)
class ModelConfig:
    axes: tuple[str, ...] = ("X", "M", "N")
    mass_kg: float = 70.0
    pitch_inertia_kg_m2: float = 26.0
    yaw_inertia_kg_m2: float = 30.0
    residual_force_limits: tuple[float, ...] = (80.0, 0.0, 0.0, 0.0, 20.0, 20.0)
    width: int = 128
    depth: int = 3

    def mass_diag(self) -> Tensor:
        return torch.tensor((self.mass_kg, self.mass_kg, self.mass_kg,
                             self.pitch_inertia_kg_m2, self.pitch_inertia_kg_m2,
                             self.yaw_inertia_kg_m2), dtype=torch.float32)

    def build(self, input_mean: Tensor, input_std: Tensor) -> PinnResidualModel:
        return PinnResidualModel(input_mean, input_std, self.mass_diag(), axis_mask(self.axes),
                                 torch.tensor(self.residual_force_limits, dtype=torch.float32),
                                 self.width, self.depth)
