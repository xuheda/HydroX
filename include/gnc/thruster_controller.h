// Copyright (c) 2026 OceanX. Author: xuheda
#pragma once
/**
 * thruster_controller.h — ThrusterVehicleController: 6-DOF control law for
 * fully/over-actuated vehicles (ROV, hovering AUV).
 *
 * Depth held by DIRECT heave thrust; heading by yaw; surge tracks a reference.
 * Produces a body wrench tau; a ThrusterMatrixAllocator maps it to thrusters.
 *
 * INERTIA NORMALISATION: gains are specified at the ACCELERATION level (per unit
 * mass, kp~wn^2 [1/s^2], kd~2*zeta*wn [1/s]) and multiplied by the vehicle mass,
 * so a 10 kg DesistekSaga and a 1863 kg RexROV2 get the same closed-loop response
 * instead of one diverging and the other not moving. (Same idea as the torpedo
 * side's apply_inertia_normalized_gains.)
 *
 * roll/pitch active gains default to ZERO: most ROVs are passively roll/pitch
 * stable (CoB above CoG), and under-actuated ones (e.g. DesistekSaga's 3 thrusters
 * = surge/yaw/heave only) cannot produce roll/pitch torque — commanding it just
 * fights the allocator. Enable per-vehicle later if a hull needs active leveling.
 */
#include "gnc/control_interfaces.h"

namespace hydrox
{
    class ThrusterVehicleController : public IController
    {
    public:
        /** Acceleration-level gains (multiplied by mass at runtime). */
        struct Axis
        {
            double kp = 0.0;        // ~ wn^2          [1/s^2] (rad: 1/s^2)
            double kd = 0.0;        // ~ 2*zeta*wn     [1/s]
            double accel_max = 0.0; // demand cap      [m/s^2] (rad: rad/s^2)
        };

        struct Params
        {
            Axis surge{0.6, 0.8, 2.0}; // X: track surge_ref or DP forward error
            Axis sway{0.6, 0.5, 1.0};  // Y: lateral damping or DP right error
            Axis heave{0.6, 1.6, 3.0}; // Z: depth hold via direct heave
            Axis roll{0.0, 0.0, 0.0};  // K: passive (off by default)
            Axis pitch{0.0, 0.0, 0.0}; // M: passive (off by default)
            Axis yaw{1.0, 1.2, 2.0};   // N: heading hold
            double mass = 11.0;        // kg — set from vehicle params (gain scaling)
        };

        explicit ThrusterVehicleController(const Params &p = {});

        void reset(const NavigationState &) override {}
        void set_mode(GNCMode mode) override { _mode = mode; }
        void set_setpoint(const GNCSetpoint &sp) override { _sp = sp; }
        Wrench update(const NavigationState &state, double dt) override;

    private:
        Params _p;
        GNCMode _mode = GNCMode::DISABLED;
        GNCSetpoint _sp;
    };

} // namespace hydrox
