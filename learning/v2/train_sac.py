"""Train and evaluate the ECA A9 low-level residual SAC v2 policy."""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import random
import sys

import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn.functional as torch_functional
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.utils import polyak_update
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

V2_ROOT = Path(__file__).resolve().parent
if str(V2_ROOT) not in sys.path:
    sys.path.insert(0, str(V2_ROOT))

from residual_env import (  # noqa: E402
    ACTION_NAMES,
    OBSERVATION_NAMES,
    EcaA9LowLevelResidualEnv,
    EnvConfig,
    ResidualLimits,
    evaluate_policy,
)


class PriorRegularizedSAC(SAC):
    """SAC with critic burn-in and a decaying physical actor prior.

    A randomly initialized critic should not be allowed to immediately destroy
    a known-stable low-level policy.  Critics therefore receive a short burn-in
    while the actor is frozen.  Afterwards the normal SAC actor objective is
    augmented by a decaying penalty to the Fossen-informed residual feedback.
    """

    def __init__(
        self,
        *args,
        prior_gain_x: float = 4.0,
        prior_gain_n: float = 4.0,
        actor_delay_updates: int = 2_000,
        prior_weight: float = 5.0,
        prior_decay_updates: int = 40_000,
        **kwargs,
    ) -> None:
        self.prior_gain_x = float(prior_gain_x)
        self.prior_gain_n = float(prior_gain_n)
        self.actor_delay_updates = int(actor_delay_updates)
        self.prior_weight = float(prior_weight)
        self.prior_decay_updates = int(prior_decay_updates)
        super().__init__(*args, **kwargs)

    def _get_constructor_parameters(self):
        data = super()._get_constructor_parameters()
        data.update({
            "prior_gain_x": self.prior_gain_x,
            "prior_gain_n": self.prior_gain_n,
            "actor_delay_updates": self.actor_delay_updates,
            "prior_weight": self.prior_weight,
            "prior_decay_updates": self.prior_decay_updates,
        })
        return data

    def _prior_actions(self, observations: torch.Tensor) -> torch.Tensor:
        surge_index = OBSERVATION_NAMES.index("surge_error")
        yaw_index = OBSERVATION_NAMES.index("yaw_rate_error")
        return torch.tanh(torch.stack((
            self.prior_gain_x * observations[:, surge_index],
            self.prior_gain_n * observations[:, yaw_index],
        ), dim=-1))

    def train(self, gradient_steps: int, batch_size: int = 64) -> None:
        self.policy.set_training_mode(True)
        optimizers = [self.actor.optimizer, self.critic.optimizer]
        if self.ent_coef_optimizer is not None:
            optimizers.append(self.ent_coef_optimizer)
        self._update_learning_rate(optimizers)

        ent_coef_losses: list[float] = []
        ent_coefs: list[float] = []
        actor_losses: list[float] = []
        critic_losses: list[float] = []
        prior_losses: list[float] = []
        prior_weights: list[float] = []
        frozen_updates = 0

        for gradient_step in range(gradient_steps):
            replay_data = self.replay_buffer.sample(
                batch_size, env=self._vec_normalize_env)
            discounts = (
                replay_data.discounts
                if replay_data.discounts is not None else self.gamma)
            if self.use_sde:
                self.actor.reset_noise()

            actions_pi, log_prob = self.actor.action_log_prob(
                replay_data.observations)
            log_prob = log_prob.reshape(-1, 1)
            ent_coef_loss = None
            if self.ent_coef_optimizer is not None and self.log_ent_coef is not None:
                ent_coef = torch.exp(self.log_ent_coef.detach())
                assert isinstance(self.target_entropy, float)
                ent_coef_loss = -(
                    self.log_ent_coef
                    * (log_prob + self.target_entropy).detach()).mean()
                ent_coef_losses.append(float(ent_coef_loss.detach().cpu()))
            else:
                ent_coef = self.ent_coef_tensor
            ent_coefs.append(float(ent_coef.detach().cpu()))
            if ent_coef_loss is not None and self.ent_coef_optimizer is not None:
                self.ent_coef_optimizer.zero_grad()
                ent_coef_loss.backward()
                self.ent_coef_optimizer.step()

            with torch.no_grad():
                next_actions, next_log_prob = self.actor.action_log_prob(
                    replay_data.next_observations)
                next_q_values = torch.cat(
                    self.critic_target(
                        replay_data.next_observations, next_actions),
                    dim=1,
                )
                next_q_values, _ = torch.min(
                    next_q_values, dim=1, keepdim=True)
                next_q_values = (
                    next_q_values
                    - ent_coef * next_log_prob.reshape(-1, 1))
                target_q_values = (
                    replay_data.rewards
                    + (1 - replay_data.dones) * discounts * next_q_values)

            current_q_values = self.critic(
                replay_data.observations, replay_data.actions)
            critic_loss = 0.5 * sum(
                torch_functional.mse_loss(current_q, target_q_values)
                for current_q in current_q_values)
            critic_losses.append(float(critic_loss.detach().cpu()))
            self.critic.optimizer.zero_grad()
            critic_loss.backward()
            self.critic.optimizer.step()

            q_values_pi = torch.cat(
                self.critic(replay_data.observations, actions_pi), dim=1)
            min_qf_pi, _ = torch.min(q_values_pi, dim=1, keepdim=True)
            sac_actor_loss = (ent_coef * log_prob - min_qf_pi).mean()
            update_index = self._n_updates + gradient_step
            if update_index < self.actor_delay_updates:
                frozen_updates += 1
                actor_losses.append(float(sac_actor_loss.detach().cpu()))
                prior_losses.append(0.0)
                prior_weights.append(self.prior_weight)
            else:
                decay_progress = (
                    (update_index - self.actor_delay_updates)
                    / max(self.prior_decay_updates, 1))
                current_prior_weight = (
                    self.prior_weight * max(0.0, 1.0 - decay_progress))
                mean_actions = self.actor(
                    replay_data.observations, deterministic=True)
                prior_actions = self._prior_actions(
                    replay_data.observations)
                prior_loss = torch_functional.mse_loss(
                    mean_actions, prior_actions)
                actor_loss = (
                    sac_actor_loss + current_prior_weight * prior_loss)
                actor_losses.append(float(actor_loss.detach().cpu()))
                prior_losses.append(float(prior_loss.detach().cpu()))
                prior_weights.append(current_prior_weight)
                self.actor.optimizer.zero_grad()
                actor_loss.backward()
                self.actor.optimizer.step()

            if gradient_step % self.target_update_interval == 0:
                polyak_update(
                    self.critic.parameters(),
                    self.critic_target.parameters(),
                    self.tau,
                )
                polyak_update(
                    self.batch_norm_stats,
                    self.batch_norm_stats_target,
                    1.0,
                )

        self._n_updates += gradient_steps
        self.logger.record("train/n_updates", self._n_updates, exclude="tensorboard")
        self.logger.record("train/ent_coef", np.mean(ent_coefs))
        self.logger.record("train/actor_loss", np.mean(actor_losses))
        self.logger.record("train/critic_loss", np.mean(critic_losses))
        self.logger.record("train/prior_loss", np.mean(prior_losses))
        self.logger.record("train/prior_weight", np.mean(prior_weights))
        self.logger.record("train/actor_frozen_fraction",
                           frozen_updates / max(gradient_steps, 1))
        if ent_coef_losses:
            self.logger.record("train/ent_coef_loss", np.mean(ent_coef_losses))


def _heuristic_prior(observation: np.ndarray, gain_x: float,
                     gain_n: float) -> np.ndarray:
    # Fixed observation indices are guarded by the exported field names.
    surge_error = observation[OBSERVATION_NAMES.index("surge_error")]
    yaw_rate_error = observation[OBSERVATION_NAMES.index("yaw_rate_error")]
    return np.tanh(np.asarray((
        gain_x * surge_error,
        gain_n * yaw_rate_error,
    ), dtype=np.float32))


def _collect_warmstart_data(config: EnvConfig, episodes: int, seed: int,
                            gain_x: float,
                            gain_n: float) -> tuple[np.ndarray, np.ndarray]:
    observations: list[np.ndarray] = []
    targets: list[np.ndarray] = []
    env = EcaA9LowLevelResidualEnv(config)
    try:
        for episode in range(episodes):
            observation, _ = env.reset(seed=seed + episode)
            done = False
            while not done:
                observations.append(observation.copy())
                targets.append(_heuristic_prior(
                    observation, gain_x, gain_n))
                observation, _reward, terminated, truncated, _info = env.step(
                    np.zeros(2, dtype=np.float32))
                done = terminated or truncated
    finally:
        env.close()
    return np.asarray(observations, dtype=np.float32), np.asarray(
        targets, dtype=np.float32)


def _warmstart_actor(model: SAC, observations: np.ndarray,
                     targets: np.ndarray, epochs: int, seed: int) -> float:
    """Fit the SAC actor mean to a stable residual-feedback prior.

    This does not replace RL.  It prevents the first replay buffer from being
    dominated by unnecessarily violent random low-level actions; SAC remains
    free to change both channels when optimizing the full trajectory reward.
    """
    if epochs <= 0 or observations.size == 0:
        return 0.0
    generator = np.random.default_rng(seed)
    device = model.device
    batch_size = min(256, len(observations))
    final_loss = 0.0
    model.actor.train()
    for _epoch in range(epochs):
        indices = generator.permutation(len(observations))
        for start in range(0, len(indices), batch_size):
            selected = indices[start:start + batch_size]
            obs_tensor = torch.as_tensor(
                observations[selected], dtype=torch.float32, device=device)
            target_tensor = torch.as_tensor(
                targets[selected], dtype=torch.float32, device=device)
            predicted = model.actor(obs_tensor, deterministic=True)
            loss = torch.nn.functional.mse_loss(predicted, target_tensor)
            model.actor.optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.actor.parameters(), 5.0)
            model.actor.optimizer.step()
            final_loss = float(loss.detach().cpu())
    # Start with moderate safe exploration around the fitted mean.  Automatic
    # entropy tuning can subsequently widen or narrow it.
    if hasattr(model.actor.log_std, "weight"):
        torch.nn.init.zeros_(model.actor.log_std.weight)
    if hasattr(model.actor.log_std, "bias"):
        torch.nn.init.constant_(model.actor.log_std.bias, -1.5)
    model.actor.eval()
    return final_loss


def _factory(config: EnvConfig, rank: int):
    def make():
        env = EcaA9LowLevelResidualEnv(config)
        return Monitor(env)
    return make


def _sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _model_policy(model: PriorRegularizedSAC):
    def policy(observation: np.ndarray) -> np.ndarray:
        return model.predict(observation, deterministic=True)[0]
    return policy


def _comparison_metrics(baseline: dict[str, float],
                        policy: dict[str, float]) -> dict[str, float | bool]:
    keys = (
        "cross_track_abs_m",
        "horizontal_error_m",
        "surge_abs_mps",
        "yaw_rate_abs_radps",
        "depth_abs_m",
        "propeller_effort",
        "saturation",
    )
    result: dict[str, float | bool] = {}
    for key in keys:
        before, after = baseline[key], policy[key]
        result[f"{key}_improvement_percent"] = (
            100.0 * (before - after) / max(abs(before), 1.0e-9))
    composite_before = (
        baseline["cross_track_abs_m"]
        + 2.0 * baseline["surge_abs_mps"]
        + 3.0 * baseline["yaw_rate_abs_radps"]
    )
    composite_after = (
        policy["cross_track_abs_m"]
        + 2.0 * policy["surge_abs_mps"]
        + 3.0 * policy["yaw_rate_abs_radps"]
    )
    result["composite_improvement_percent"] = (
        100.0 * (composite_before - composite_after)
        / max(composite_before, 1.0e-9))
    result["passes_effectiveness_gate"] = bool(
        result["composite_improvement_percent"] >= 5.0
        and policy["hard_termination"] <= baseline["hard_termination"]
        and policy["saturation"] <= max(1.10 * baseline["saturation"], 0.20)
        and policy["depth_abs_m"] <= 1.10 * baseline["depth_abs_m"]
    )
    return result


def _plot_comparison(output: Path,
                     baseline_trajectories: list[dict[str, np.ndarray]],
                     policy_trajectories: list[dict[str, np.ndarray]],
                     filename: str = "baseline_vs_residual.png") -> None:
    baseline = baseline_trajectories[0]
    policy = policy_trajectories[0]
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    axis = axes[0, 0]
    axis.plot(baseline["reference_east_m"], baseline["reference_north_m"],
              "k--", linewidth=1.5, label="reference")
    axis.plot(baseline["east_m"], baseline["north_m"],
              linewidth=1.4, label="classical")
    axis.plot(policy["east_m"], policy["north_m"],
              linewidth=1.4, label="residual SAC")
    axis.set_title("Horizontal trajectory")
    axis.set_xlabel("East (m)")
    axis.set_ylabel("North (m)")
    axis.axis("equal")
    axis.grid(True, alpha=0.3)
    axis.legend()

    axis = axes[0, 1]
    axis.plot(baseline["time_s"], baseline["cross_track_error_m"],
              label="classical")
    axis.plot(policy["time_s"], policy["cross_track_error_m"],
              label="residual SAC")
    axis.set_title("Absolute cross-track error")
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Error (m)")
    axis.grid(True, alpha=0.3)
    axis.legend()

    axis = axes[1, 0]
    axis.plot(baseline["time_s"], baseline["surge_error_mps"],
              label="classical")
    axis.plot(policy["time_s"], policy["surge_error_mps"],
              label="residual SAC")
    axis.set_title("Absolute surge-speed error")
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Error (m/s)")
    axis.grid(True, alpha=0.3)
    axis.legend()

    axis = axes[1, 1]
    axis.plot(policy["time_s"], policy["residual_x_n"], label="delta X (N)")
    axis.plot(policy["time_s"], policy["residual_n_nm"], label="delta N (N m)")
    axis.set_title("Applied guarded residual")
    axis.set_xlabel("Time (s)")
    axis.set_ylabel("Residual")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.savefig(output / filename, dpi=160)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Train causal HydroX-in-the-loop ECA A9 residual SAC v2")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timesteps", type=int, default=100_000)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=20260730)
    parser.add_argument("--eval-episodes", type=int, default=12)
    parser.add_argument("--episode-s", type=float, default=45.0)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--shadow-executable", type=Path)
    parser.add_argument("--vehicle-parameters", type=Path)
    parser.add_argument("--resume", type=Path)
    parser.add_argument("--warmstart-episodes", type=int, default=4)
    parser.add_argument("--warmstart-epochs", type=int, default=12)
    parser.add_argument("--warmstart-gain-x", type=float, default=4.0)
    parser.add_argument("--warmstart-gain-n", type=float, default=4.0)
    args = parser.parse_args()
    if (args.timesteps <= 0 or args.n_envs <= 0 or args.eval_episodes <= 0
            or args.warmstart_episodes < 0 or args.warmstart_epochs < 0):
        raise ValueError("timesteps, n-envs and eval-episodes must be positive")

    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    args.output.mkdir(parents=True, exist_ok=True)
    defaults = EnvConfig()
    config = EnvConfig(
        episode_s=args.episode_s,
        shadow_executable=str(args.shadow_executable or defaults.shadow_executable),
        vehicle_parameters=str(args.vehicle_parameters or defaults.vehicle_parameters),
        limits=ResidualLimits(),
    )

    factories = [_factory(config, rank) for rank in range(args.n_envs)]
    train_env = (
        SubprocVecEnv(factories, start_method="spawn")
        if args.n_envs > 1 else DummyVecEnv(factories)
    )
    eval_config = EnvConfig(
        episode_s=args.episode_s,
        domain_randomization=True,
        observation_noise_scale=0.0,
        shadow_executable=config.shadow_executable,
        vehicle_parameters=config.vehicle_parameters,
        limits=config.limits,
    )
    callback_env = DummyVecEnv([_factory(eval_config, 10_000)])
    callback = EvalCallback(
        callback_env,
        best_model_save_path=str(args.output / "best_model"),
        log_path=str(args.output / "evaluations"),
        eval_freq=max(5_000 // args.n_envs, 1),
        n_eval_episodes=4,
        deterministic=True,
        render=False,
    )

    if args.resume:
        model = PriorRegularizedSAC.load(
            str(args.resume), env=train_env, device=args.device)
        warmstart_samples = 0
        warmstart_loss = None
    else:
        model = PriorRegularizedSAC(
            "MlpPolicy",
            train_env,
            learning_rate=3.0e-4,
            buffer_size=300_000,
            learning_starts=max(4_000, 1_000 * args.n_envs),
            batch_size=256,
            tau=0.005,
            gamma=0.995,
            train_freq=(1, "step"),
            gradient_steps=1,
            ent_coef="auto",
            policy_kwargs={"net_arch": [256, 256]},
            prior_gain_x=args.warmstart_gain_x,
            prior_gain_n=args.warmstart_gain_n,
            actor_delay_updates=2_000,
            prior_weight=5.0,
            prior_decay_updates=40_000,
            seed=args.seed,
            device=args.device,
            verbose=1,
        )
        warm_observations, warm_targets = _collect_warmstart_data(
            eval_config,
            args.warmstart_episodes,
            args.seed + 20_000,
            args.warmstart_gain_x,
            args.warmstart_gain_n,
        )
        warmstart_samples = len(warm_observations)
        warmstart_loss = _warmstart_actor(
            model,
            warm_observations,
            warm_targets,
            args.warmstart_epochs,
            args.seed,
        )
        model.save(str(args.output / "physics_prior_initialization"))
    model.learn(
        total_timesteps=args.timesteps,
        callback=callback,
        progress_bar=False,
        reset_num_timesteps=args.resume is None,
    )
    model.save(str(args.output / "last_model"))
    best_path = args.output / "best_model" / "best_model.zip"
    selected_model = (
        PriorRegularizedSAC.load(str(best_path), device=args.device)
        if best_path.is_file() else model
    )
    model_path = args.output / "low_level_residual_sac_v2"
    selected_model.save(str(model_path))

    seeds = [args.seed + 50_000 + index for index in range(args.eval_episodes)]
    factory = lambda: EcaA9LowLevelResidualEnv(eval_config)
    baseline, baseline_trajectories = evaluate_policy(
        factory, lambda _observation: np.zeros(2, dtype=np.float32), seeds)
    prior, _prior_trajectories = evaluate_policy(
        factory,
        lambda observation: _heuristic_prior(
            observation, args.warmstart_gain_x, args.warmstart_gain_n),
        seeds,
    )
    policy, policy_trajectories = evaluate_policy(
        factory, _model_policy(selected_model), seeds)
    comparison = _comparison_metrics(baseline, policy)
    _plot_comparison(args.output, baseline_trajectories, policy_trajectories)

    contract = {
        "format": "hydrox.low-level-residual-sac/v2",
        "deployment_authority": "none",
        "vehicle": "EcaA9",
        "policy_hz": config.policy_hz,
        "control_hz": config.control_hz,
        "observation_names": list(OBSERVATION_NAMES),
        "action_names": list(ACTION_NAMES),
        "action_contract": "tau_final=tau_base+[delta_X,0,0,0,0,delta_N]",
        "classical_pitch_depth_channel": True,
        "training_transition": (
            "HydroX C++ GNC+ResidualSafetyFilter+FinAllocator -> "
            "OceanX-parity six-DOF Fossen"),
        "environment": asdict(config),
        "vehicle_parameters_sha256": _sha256(config.vehicle_parameters),
        "shadow_executable_sha256": _sha256(config.shadow_executable),
        "seed": args.seed,
        "timesteps": args.timesteps,
        "n_envs": args.n_envs,
        "initialization": {
            "method": "Fossen-informed residual-feedback behavior cloning",
            "samples": warmstart_samples,
            "epochs": args.warmstart_epochs if not args.resume else 0,
            "final_mse": warmstart_loss,
            "gain_x": args.warmstart_gain_x,
            "gain_n": args.warmstart_gain_n,
            "note": (
                "The prior only initializes the actor mean. SAC subsequently "
                "optimizes the complete stochastic policy and critics."
            ),
        },
        "baseline": baseline,
        "physics_prior": prior,
        "policy": policy,
        "comparison": comparison,
        "acceptance_gate": {
            "minimum_composite_improvement_percent": 5.0,
            "no_hard_termination_regression": True,
            "maximum_saturation_increase_percent": 10.0,
            "maximum_depth_error_increase_percent": 10.0,
        },
        "next_gate": (
            "Run the frozen policy in OceanX SITL shadow mode on unseen current "
            "fields; this artifact has no actuator authority."),
    }
    (args.output / "evaluation.json").write_text(
        json.dumps(contract, indent=2), encoding="utf-8")
    (args.output / "observation_contract.json").write_text(
        json.dumps({
            "format": contract["format"],
            "observation_names": contract["observation_names"],
            "action_names": contract["action_names"],
            "normalization": "fixed physical scales compiled in residual_env.py",
        }, indent=2), encoding="utf-8")
    train_env.close()
    callback_env.close()
    print(json.dumps(contract, indent=2))


if __name__ == "__main__":
    main()
