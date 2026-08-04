# HydroX low-level residual RL design

> **v2 decision (2026-07-30):** the first candidate controls only `delta_X`
> and `delta_N`.  It trains through the original HydroX C++ controller,
> residual safety filter and fin allocator at 100 Hz, followed by the OceanX
> ECA A9 six-DOF Fossen plant.  The earlier PINN-only transition environment is
> retained as a model-identification experiment, not as the authoritative RL
> environment.  See `learning/v2/README.md`.

## 1. Goal and safety boundary

This project adds a learned **low-level residual** to HydroX without replacing
the EKF, GNC controller, allocator, or actuator protections.

```text
EKF state + GNC setpoint ─┐
                          ├─ classical controller ──> tau_base ─┐
                          │                                      ├─ allocator ─> actuators
                          └─ RL policy ────────────> delta_tau ──┘
                                      ^                safety filter
                                      │
                            PINN residual world model
```

The deployed action is always:

```text
tau_final = tau_base + alpha * clip_rate(delta_tau_RL)
```

`alpha` is zero by default. Invalid inference, low confidence, an out-of-range
observation, saturation, missed inference deadline, or an operator disable
sets `alpha = 0` for that control tick. The result is exactly the current
HydroX control path.

The learned policy never writes MAVLink actuator channels and never bypasses
`IAllocator`.

## 2. Chosen learning problem

### AUV v1

The slender-body AUV is underactuated. Its policy action has only three
normalized components:

```text
a = [a_X, a_M, a_N] in [-1, 1]^3
delta_tau = [a_X * limit_X, 0, 0, 0, a_M * limit_M, a_N * limit_N]
```

They compensate longitudinal force, pitch moment, and yaw moment respectively.
There is deliberately no direct heave correction for a fin-driven AUV.

### ROV v2

For a fully actuated ROV, enable the same mechanism axis by axis:

```text
a = [a_X, a_Y, a_Z, a_K, a_M, a_N] in [-1, 1]^6
```

ROV deployment starts with `X/Y/Z/N`; roll and pitch corrections require
separate validation because their safety margin is smaller.

## 3. Observation contract

All inputs are SI units in body-FRD / world-NED as already used by HydroX.
Angles are represented by sine and cosine; raw wrapped yaw is never fed to a
network.

| Group | AUV v1 fields |
| --- | --- |
| Estimated motion | `nu[0..5]`, depth, `sin/cos(roll,pitch,yaw)` |
| Control objective | depth error, surge error, yaw-rate error, `sin/cos(heading error)`, GNC mode |
| Environment | EKF current N/E/D, DVL validity/age, pose/twist covariance traces |
| Actuation history | latest 5 actuator command vectors and their first differences |
| Runtime health | actuator saturation ratio, normalized battery voltage/power when available |

The policy runs at 20 Hz. The existing GNC/allocator remains at 100 Hz. The
latest policy action is held between policy ticks and is still subject to the
per-tick safety rate limit.

`normalization.json` is a deployment artifact. It specifies field order,
mean, standard deviation, hard valid ranges, action axes, model version, and
a SHA-256 checksum for the paired model. C++ must reject a model whose contract
does not exactly match its compiled observation version.

## 4. PINN residual world model

The PINN is a grey-box dynamics model, not an actuator controller. It models
the unmodelled generalized force:

```text
M * nu_dot + C(nu) * nu + D(nu) * nu + g(eta)
    = tau_actuator + delta_tau_PINN(x, u)
```

`delta_tau_PINN` is predicted as a mean and a diagonal log variance. Five
independently seeded models form an ensemble; model disagreement is the
uncertainty used by the RL trainer and deployment gate.

The neural residual is constrained to be zero at rest with zero command. The
loss is:

```text
L = L_one_step
  + lambda_phys * L_dynamics_residual
  + lambda_diss * max(0, nu^T * delta_tau_PINN - power_allowance)^2
  + lambda_zero * ||delta_tau_PINN(0, 0)||^2
  + lambda_smooth * ||delta_tau(t) - delta_tau(t-dt)||^2
```

Initially fit only `X/M/N` for an AUV. This is data efficient and matches the
actual controller authority. The Python `fossen_core` starts with vehicle
bundle mass, pitch inertia, fin geometry, and propeller limits; hydrodynamic
coefficients are either fitted or domain-randomized. HydroX's current control
parameters are not treated as a complete 6-DOF simulator model.

## 5. RL training design

Use **Soft Actor-Critic (SAC)** with a three-dimensional, tanh-bounded action.
SAC is selected because the action is continuous and its stochastic policy
encourages exploration during SITL training. Training is entirely Python.

The environment transition is the Fossen core plus a sampled PINN ensemble
member. Episode randomization covers initial condition, current, sensor delay,
DVL dropouts, mass/drag/thrust scaling, actuator delay, and bounded fin/prop
effectiveness loss. The classical HydroX controller is part of every rollout;
RL learns only the residual.

```text
reward =
  - w_track * tracking_error
  - w_energy * estimated_power
  - w_smooth * ||a_t - a_t-1||^2
  - w_sat * actuator_saturation
  - w_uncertain * PINN_ensemble_variance
  - w_unsafe * safety_violation
```

Termination is triggered by hard depth/attitude limits, stale-estimator
conditions, prolonged saturation, collision, or simulation fault. A fallback
run with `a = 0` is always evaluated beside each learned rollout.

## 6. Data and dataset rules

Current XLog already contains state, setpoint, controller wrench, actuator
commands, timing, estimator health, and optional simulator truth. This is the
primary dataset source.

1. Decode by XLog schema, never by hard-coded byte offsets alone.
2. Join records by timestamp and retain only monotonic, finite time windows.
3. Build derivatives with a smoothing differentiator; do not finite-difference
   raw 100 Hz velocity without filtering.
4. Prefer `SimulatorTruth` for early SITL PINN labels. Train a separate
   sensor-realistic validation split using EKF state.
5. Split by **run**, vehicle configuration, and current scenario, never by
   random adjacent rows. This prevents temporal leakage.
6. Record source XLog hashes and bundle version in every dataset manifest.

The ignored `learning/datasets/` directory holds only local manifests and
derived arrays. Large XLogs/checkpoints do not enter Git.

## 7. Deployment phases

| Phase | Output | Flight-control authority |
| --- | --- | --- |
| 0: collector | Reproducible XLog dataset manifest | None |
| 1: PINN shadow | One-step prediction and uncertainty report | None |
| 2: RL SITL | Candidate ONNX policy, offline metrics | None |
| 3: SITL shadow | C++/Python policy action recorded next to `tau_base` | None |
| 4: guarded SITL | `alpha <= 0.10`, all safety gates active | Small |
| 5: HIL / water tests | Per-scenario approval and rollback test | Small, reviewed |

No phase advances based solely on mean reward. It must meet validation and
fallback criteria below.

## 8. Acceptance gates

Before guarded SITL authority:

- PINN one-step state error is lower than the pure physics baseline on unseen
  current and payload scenarios.
- No episode has a worse hard safety violation rate than the classical
  controller baseline.
- 99.9th-percentile inference time is below 5 ms at 20 Hz on the deployment
  compute target.
- Policy action remains in range before C++ clipping for at least 99.99% of
  validation steps.
- C++ safety-filter tests cover disabled mode, non-finite action, low
  confidence, each-axis limit, and rate limit.
- Shadow replay shows that guarded `tau_final` improves the chosen metric
  without materially increasing actuator saturation or energy use.

## 9. Artifact and runtime contract

Training exports a versioned directory:

```text
run-YYYYMMDD-HHMM/
  pinn_ensemble/               # Python-only training checkpoint
  residual_policy.onnx          # optional deployment model
  normalization.json            # required input/action contract
  limits.json                   # reviewed per-axis force/moment and slew limits
  evaluation.json               # metrics, seed, dataset manifest hashes
  SHA256SUMS
```

The first integration uses a Python DDS shadow policy. ONNX Runtime is only
added to the C++ build after the same exported model has passed replay parity:
for a frozen XLog input stream, Python and C++ actions must agree within a
defined numeric tolerance. The existing `ResidualRlModule` is intentionally
an interface only; it should receive an ONNX-backed `IResidualPolicy` only
after this parity test exists.

## 10. Planned local Python layout

```text
learning/
  training/
    xlog_dataset.py             # schema-aware XLog -> trajectory windows
    fossen_core.py              # differentiable physics baseline
    pinn_residual.py            # ensemble PINN training
    residual_env.py             # SAC environment
    train_sac.py
  evaluation/
    evaluate_pinn.py
    replay_shadow.py
    report.py
  export/
    export_onnx.py
    validate_contract.py
```

The next implementation increment is Phase 0: a schema-aware XLog decoder and
dataset manifest generator, followed by a baseline physics-only evaluation.
