from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from ecaa9_fossen import DomainSample, EcaA9FossenPlant, EcaA9Parameters
from residual_env import (
    OBSERVATION_NAMES,
    EcaA9LowLevelResidualEnv,
    EnvConfig,
)


class LowLevelV2Tests(unittest.TestCase):
    def test_positive_propeller_command_accelerates_vehicle(self) -> None:
        config = EnvConfig(domain_randomization=False, observation_noise_scale=0.0)
        params = EcaA9Parameters.load(config.vehicle_parameters)
        plant = EcaA9FossenPlant(params)
        eta = np.asarray((0.0, 0.0, 100.0, 0.0, 0.0, 0.0))
        plant.reset(eta, np.zeros(6), DomainSample())
        controls = np.zeros(8)
        controls[4] = 0.8
        for _ in range(100):
            plant.step(controls, 0.01)
        self.assertGreater(plant.nu[0], 0.15)
        self.assertGreater(plant.last_applied_wrench[0], 0.0)

    def test_residual_action_is_causal_and_pitch_remains_classical(self) -> None:
        config = EnvConfig(
            episode_s=1.0, domain_randomization=False,
            observation_noise_scale=0.0)
        baseline = EcaA9LowLevelResidualEnv(config)
        residual = EcaA9LowLevelResidualEnv(config)
        try:
            baseline_obs, _ = baseline.reset(seed=91)
            residual_obs, _ = residual.reset(seed=91)
            self.assertEqual(
                baseline_obs.shape, (len(OBSERVATION_NAMES),))
            self.assertEqual(residual_obs.shape, baseline_obs.shape)
            self.assertTrue(np.allclose(baseline.plant.eta, residual.plant.eta))
            _, _, _, _, baseline_info = baseline.step(
                np.zeros(2, dtype=np.float32))
            _, _, _, _, residual_info = residual.step(
                np.asarray((1.0, -1.0), dtype=np.float32))
            self.assertIsNotNone(residual.last_shadow)
            self.assertEqual(residual.last_shadow.applied_delta[1], 0.0)
            self.assertGreater(
                residual_info["residual_x_n"], baseline_info["residual_x_n"])
            self.assertLess(
                residual_info["residual_n_nm"], baseline_info["residual_n_nm"])
            self.assertFalse(np.allclose(baseline.plant.nu, residual.plant.nu))
        finally:
            baseline.close()
            residual.close()

    def test_environment_rollout_is_finite(self) -> None:
        env = EcaA9LowLevelResidualEnv(EnvConfig(
            episode_s=2.0, observation_noise_scale=0.0))
        try:
            observation, info = env.reset(seed=123)
            self.assertEqual(info["action_axes"], ("delta_X", "delta_N"))
            self.assertTrue(np.isfinite(observation).all())
            done = False
            steps = 0
            while not done:
                observation, reward, terminated, truncated, step_info = env.step(
                    np.zeros(2, dtype=np.float32))
                self.assertTrue(np.isfinite(observation).all())
                self.assertTrue(np.isfinite(reward))
                done = terminated or truncated
                steps += 1
            self.assertFalse(step_info["hard_termination"])
            self.assertEqual(steps, 40)
        finally:
            env.close()


if __name__ == "__main__":
    unittest.main()
