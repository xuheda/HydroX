# Start low-level residual RL v2

The current training entry point is `learning/v2/train_sac.py`.  It closes the
action loop through the original HydroX C++ controller/allocator and the
OceanX ECA A9 six-degree-of-freedom Fossen model.  Read
`learning/v2/README.md` for the exact action contract and acceptance gates.

The older PINN/SAC pilot instructions below are retained for reproducibility,
but that environment is not a deployment candidate because its horizontal and
depth transitions were simplified.

# Legacy v1 PINN pilot

The learning workspace is local-only (`learning/` is ignored by Git). Use the
existing `py310` Conda environment. Its interpreter is
`C:\Users\Administrator\miniconda3\envs\py310\python.exe`; define the
following once per PowerShell session if `conda activate` is unavailable on
your `PATH`:

```powershell
$Py310 = 'C:\Users\Administrator\miniconda3\envs\py310\python.exe'
```

The current repository contains the complete dataset builder and PINN trainer,
but there are no `.xlog` files in the workspace yet. Generate baseline data
before trying to train an RL policy.

## 1. Collect baseline SITL runs

Keep learned residual control disabled. Run the ordinary controller through a
range of depth, speed, heading/yaw-rate, current, payload, and sensor-health
scenarios. Store each run separately.

```powershell
New-Item -ItemType Directory -Force learning\datasets\raw | Out-Null

.\build_sitl\Release\hydrox_sitl.exe `
  --vehicle-bundle profiles/generic-auv-fin/vehicle-bundle.json `
  --xlog learning/datasets/raw/auv_baseline_001.xlog
```

For a useful first model, collect at least 20 independent runs, not one long
run. Include currents in both directions, turns at several speeds, depth
changes, and a limited amount of propulsion/fin effectiveness variation.

Do not use truth-heading aid as the only dataset. It is useful for an early
SITL label set, but the validation set must also contain the ordinary EKF path.

## 2. Build a reproducible transition dataset

```powershell
& $Py310 learning\training\xlog_dataset.py `
  "learning/datasets/raw/*.xlog" `
  --output learning/datasets/auv_v1.npz
```

This writes `auv_v1.npz` and a JSON manifest with source runs and feature
order. Inspect the source list before training; failed or repeated scenarios
should be removed before retraining.

## 3. Train the five-member PINN ensemble

The supplied mass and pitch inertia match
`profiles/generic-auv-fin/vehicle-bundle.json`. Replace them for another
vehicle. `X/M/N` is intentional: it matches the fin AUV's actual control
authority.

```powershell
& $Py310 learning\training\train_pinn.py `
  learning/datasets/auv_v1.npz `
  --output learning/export/auv_pinn_001 `
  --epochs 200 `
  --ensemble 5 `
  --axes X,M,N `
  --mass-kg 70 `
  --pitch-inertia 26 `
  --yaw-inertia 30
```

Read `learning/export/auv_pinn_001/pinn_metrics.json`. The model must improve
on the pure prior **on held-out runs**, not merely the training data. Do not
train or deploy residual RL if the held-out `model_mse` is not lower than
`baseline_mse` consistently across the ensemble.

## 4. Next gate: RL shadow mode

After the PINN passes its held-out baseline, use it as the model environment
for SAC residual-RL experiments. The first policy runs in shadow mode only:
its candidate `delta_tau` is logged and compared with `tau_base`; it has no
authority over the allocator or actuators.

Only after replay parity, policy latency, saturation, and safety gates pass
may `HYDROX_ENABLE_RESIDUAL_RL` be compiled and its reviewed per-axis limits
enabled. The default production build remains unchanged.
