// Copyright (c) 2026 OceanX. Author: xuheda
#pragma once
/**
 * control_interfaces.h — the tau-contract that decouples control law from
 * actuator layout. See doc/hydrox_control_architecture.md.
 *
 *   IController  : produces a body wrench tau (the only thing a control law knows)
 *   IAllocator   : maps tau -> normalised actuator channels (the only thing a
 *                  layout knows)
 *
 * A vehicle = one IController archetype + one IAllocator layout, selected by the
 * vehicle profile. Same-archetype vehicles differ only by params; new layouts
 * are new IAllocator impls (matrix-driven), not per-vehicle code.
 */
#include "types.h" // AUVState, GNCSetpoint, GNCMode, FinCmd
#include <Eigen/Core>
#include <array>

namespace hydrox
{
    /** Generalised body force/moment: tau = [X, Y, Z, K, M, N] (N, N·m), body NED. */
    using Wrench = Eigen::Matrix<double, 6, 1>;

    /**
     * Normalised actuator command handed to the MAVLink HIL_ACTUATOR_CONTROLS encoder.
     * 8 channels cover both layouts: fin vehicles use ch[0..4] (4 fins + thrust);
     * thruster vehicles use ch[0..n-1] (up to 8 thrusters). MAVLink carries 10 slots.
     */
    struct ActuatorCmd
    {
        std::array<float, 8> ch{}; // normalised [-1,1] MAVLink channels
        double rpm = 0.0;          // primary shaft speed (RPM) for local MotorModel + telemetry (fin/prop only)
    };

    /** Control law: state + setpoint -> desired body wrench (only achievable DOFs). */
    struct IController
    {
        virtual ~IController() = default;
        virtual void reset(const AUVState &state) = 0;
        virtual void set_mode(GNCMode mode) = 0;
        virtual void set_setpoint(const GNCSetpoint &sp) = 0;
        virtual Wrench update(const AUVState &state, double dt) = 0;
    };

    /** Control allocation: desired wrench -> normalised actuator commands. */
    struct IAllocator
    {
        virtual ~IAllocator() = default;
        virtual ActuatorCmd allocate(const Wrench &tau, double surge) const = 0;
    };

} // namespace hydrox
