# ECA A9 low-level residual RL v2

This directory replaces the old PINN-only transition surrogate for the first
deployable learning candidate.  It does **not** replace HydroX control:

```text
reference / estimated state
        |
        +--> original HydroX C++ GNC --> tau_base --------+
        |                                                 |
        +--> SAC policy --> guarded [delta_X, delta_N] ----+
                                                          |
                                    original FinAllocator |
                                                          v
                                            fins + propeller
                                                          |
                                OceanX ECA A9 6-DOF Fossen
```

The important difference from v1 is causal closure.  One policy action is held
for 50 ms, but the C++ controller, safety filter and allocator run at 100 Hz
inside that interval.  Their actual actuator channels drive the same ECA A9
parameter file used by OceanX.  Pitch/depth (`M`) is classical-only.

## Acceptance target

A candidate is useful only if the fixed-seed evaluation reports:

* at least 5% improvement in the composite cross-track/surge/yaw-rate metric;
* no increase in hard termination rate;
* no more than 10% increase in actuator saturation;
* no more than 10% degradation in depth error.

Passing this local gate still grants no actuator authority.  The next test is a
frozen-policy OceanX SITL shadow run on unseen current fields.

## Run

Use the existing `py310` Conda environment:

```powershell
$Py310 = 'C:\Users\Administrator\miniconda3\envs\py310\python.exe'

& $Py310 learning\v2\test_low_level_v2.py -v

& $Py310 learning\v2\train_sac.py `
  --output learning\export\ecaa9_low_level_v2_001 `
  --timesteps 100000 `
  --n-envs 4 `
  --eval-episodes 12

& $Py310 learning\v2\evaluate_candidate.py `
  learning\export\ecaa9_low_level_v2_001\low_level_residual_sac_v2.zip `
  --output learning\export\ecaa9_low_level_v2_001 `
  --episodes 12
```

For a wiring-only smoke run use `--timesteps 5000 --n-envs 1
--eval-episodes 2`.  Its score is not an effectiveness claim.

The trainer uses Guided SAC: a Fossen-informed actor warm start, a critic-only
burn-in, then the normal SAC actor objective plus a decaying prior penalty.
The penalty prevents an untrained critic from immediately destroying a known
stable inner-loop policy; it decays so RL can improve beyond the prior.
