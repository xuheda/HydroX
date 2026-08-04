"""Smoke test for the frozen-PINN residual SAC environment."""
from __future__ import annotations

import glob

import numpy as np

from residual_env import PinnEnsemble, ResidualAuvEnv


def main() -> None:
    checkpoints = sorted(glob.glob("learning/export/ecaa9_pinn_pretrain_001/pinn_member_*.pt"))
    env = ResidualAuvEnv(PinnEnsemble(checkpoints))
    observation, _ = env.reset(seed=7)
    for _ in range(40):
        observation, _reward, terminated, truncated, _info = env.step(np.zeros(3, dtype=np.float32))
        assert np.isfinite(observation).all()
        assert not terminated
        assert not truncated
    assert observation.shape == env.observation_space.shape
    print("residual_env smoke test: PASS")


if __name__ == "__main__":
    main()
