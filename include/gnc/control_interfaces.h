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
#include "types.h" // NavigationState, GNCSetpoint, GNCMode
#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace hydrox
{
    /** Generalised body force/moment: tau = [X, Y, Z, K, M, N] (N, N·m), body FRD. */
    using Wrench = Eigen::Matrix<double, 6, 1>;

    enum class ActuatorLayout : std::uint8_t
    {
        Unspecified = 0,
        FinAndPropeller,
        ThrusterArray,
        DirectBodyWrench,
        TwinScrew,
        QuadX,
        ConventionalFixedWing,
        LiftCruiseVtol,
        DifferentialDrive,
    };

    enum class ActuatorQuantity : std::uint8_t
    {
        Unused = 0,
        Thrust,
        Position,
        AngularVelocity,
        BodyForce,
        BodyMoment,
    };

    struct ActuatorChannelContract
    {
        ActuatorQuantity quantity = ActuatorQuantity::Unused;
        bool reversible = false;
    };

    /**
     * Normalised actuator command handed to the MAVLink HIL_ACTUATOR_CONTROLS encoder.
     * HydroX populates up to eight logical channels; MAVLink transports sixteen.
     * The value is dimensionless, while contract[] records what was normalised.
     * Canonical meanings are thrust T/T_max, position delta/delta_max, and
     * angular velocity omega/omega_max. Conversion happens only in the setters.
     */
    struct ActuatorCmd
    {
        std::array<float, 8> ch{};
        std::array<ActuatorChannelContract, 8> contract{};
        ActuatorLayout layout = ActuatorLayout::Unspecified;
        std::uint8_t active_count = 0;
        double rpm = 0.0;

        void set_thrust(std::size_t index, double thrust_N,
                        double max_forward_N, double max_reverse_N = 0.0)
        {
            const bool reversible = max_reverse_N > 0.0;
            set_asymmetric(index, thrust_N, max_forward_N, max_reverse_N,
                           ActuatorQuantity::Thrust, reversible);
        }

        void set_position(std::size_t index, double position_rad,
                          double max_positive_rad, double max_negative_rad)
        {
            set_asymmetric(index, position_rad, max_positive_rad,
                           max_negative_rad, ActuatorQuantity::Position, true);
        }

        void set_angular_velocity(std::size_t index,
                                  double angular_velocity_radps,
                                  double max_forward_radps,
                                  double max_reverse_radps = 0.0)
        {
            const bool reversible = max_reverse_radps > 0.0;
            set_asymmetric(index, angular_velocity_radps, max_forward_radps,
                           max_reverse_radps,
                           ActuatorQuantity::AngularVelocity, reversible);
        }

        void set_body_force(std::size_t index, double force_N, double limit_N)
        {
            set_asymmetric(index, force_N, limit_N, limit_N,
                           ActuatorQuantity::BodyForce, true);
        }

        void set_body_moment(std::size_t index, double moment_Nm, double limit_Nm)
        {
            set_asymmetric(index, moment_Nm, limit_Nm, limit_Nm,
                           ActuatorQuantity::BodyMoment, true);
        }

        bool valid() const noexcept
        {
            if (active_count > ch.size())
                return false;
            switch (layout)
            {
            case ActuatorLayout::Unspecified:
                if (active_count != 0) return false;
                break;
            case ActuatorLayout::FinAndPropeller:
                if (active_count != 5) return false;
                break;
            case ActuatorLayout::ThrusterArray:
                if (active_count == 0) return false;
                break;
            case ActuatorLayout::DirectBodyWrench:
                if (active_count != 6) return false;
                break;
            case ActuatorLayout::TwinScrew:
            case ActuatorLayout::DifferentialDrive:
                if (active_count != 2) return false;
                break;
            case ActuatorLayout::QuadX:
            case ActuatorLayout::ConventionalFixedWing:
                if (active_count != 4) return false;
                break;
            case ActuatorLayout::LiftCruiseVtol:
                if (active_count != 8) return false;
                break;
            }
            const auto expected_contract = [this](std::size_t index)
            {
                if (layout == ActuatorLayout::DirectBodyWrench)
                    return ActuatorChannelContract{
                        index < 3 ? ActuatorQuantity::BodyForce
                                  : ActuatorQuantity::BodyMoment,
                        true};
                if (layout == ActuatorLayout::DifferentialDrive)
                    return ActuatorChannelContract{
                        ActuatorQuantity::AngularVelocity, true};
                if (layout == ActuatorLayout::FinAndPropeller)
                    return ActuatorChannelContract{
                        index < 4 ? ActuatorQuantity::Position
                                  : ActuatorQuantity::Thrust,
                        true};
                if (layout == ActuatorLayout::ThrusterArray ||
                    layout == ActuatorLayout::TwinScrew)
                    return ActuatorChannelContract{ActuatorQuantity::Thrust, true};
                if (layout == ActuatorLayout::ConventionalFixedWing)
                    return ActuatorChannelContract{
                        index < 3 ? ActuatorQuantity::Position
                                  : ActuatorQuantity::Thrust,
                        index < 3};
                if (layout == ActuatorLayout::LiftCruiseVtol)
                    return ActuatorChannelContract{
                        index >= 4 && index <= 6 ? ActuatorQuantity::Position
                                                : ActuatorQuantity::Thrust,
                        index >= 4 && index <= 6};
                return ActuatorChannelContract{ActuatorQuantity::Thrust, false};
            };
            for (std::size_t index = 0; index < ch.size(); ++index)
            {
                if (!std::isfinite(ch[index]))
                    return false;
                if (index >= active_count)
                {
                    if (std::abs(ch[index]) > 1.0e-6f ||
                        contract[index].quantity != ActuatorQuantity::Unused)
                        return false;
                    continue;
                }
                const ActuatorChannelContract expected = expected_contract(index);
                if (contract[index].quantity != expected.quantity ||
                    contract[index].reversible != expected.reversible)
                {
                    return false;
                }
                const float lower = contract[index].reversible ? -1.0f : 0.0f;
                if (ch[index] < lower - 1.0e-6f || ch[index] > 1.0f + 1.0e-6f)
                    return false;
            }
            return std::isfinite(rpm);
        }

    private:
        void set_asymmetric(std::size_t index, double physical_value,
                            double max_positive, double max_negative,
                            ActuatorQuantity quantity, bool reversible)
        {
            if (index >= ch.size())
                return;
            contract[index] = {quantity, reversible};
            active_count = static_cast<std::uint8_t>(
                std::max<std::size_t>(active_count, index + 1));
            const bool limits_valid =
                std::isfinite(max_positive) && max_positive > 0.0 &&
                (!reversible ||
                 (std::isfinite(max_negative) && max_negative > 0.0));
            if (!std::isfinite(physical_value) || !limits_valid)
            {
                ch[index] = std::numeric_limits<float>::quiet_NaN();
                return;
            }
            const double positive_limit = max_positive;
            const double negative_limit = reversible ? max_negative : 1.0;
            const double normalized = physical_value >= 0.0
                                          ? physical_value / positive_limit
                                          : (reversible ? physical_value / negative_limit : 0.0);
            const double lower = reversible ? -1.0 : 0.0;
            ch[index] = static_cast<float>(std::clamp(normalized, lower, 1.0));
        }
    };

    enum class FlightPhase : uint8_t { NotApplicable, Hover, FrontTransition, Cruise, BackTransition };

    /** Control law: state + setpoint -> desired body wrench (only achievable DOFs). */
    struct IController
    {
        virtual ~IController() = default;
        virtual void reset(const NavigationState &state) = 0;
        virtual void set_mode(GNCMode mode) = 0;
        virtual void set_setpoint(const GNCSetpoint &sp) = 0;
        virtual Wrench update(const NavigationState &state, double dt) = 0;
        virtual FlightPhase flight_phase() const noexcept { return FlightPhase::NotApplicable; }
        virtual bool transition_fault() const noexcept { return false; }
        virtual bool acknowledge_transition_fault() noexcept { return true; }
    };

    /** Control allocation: desired wrench -> normalised actuator commands. */
    struct IAllocator
    {
        virtual ~IAllocator() = default;
        virtual ActuatorCmd allocate(const Wrench &tau, double surge) const = 0;
    };

} // namespace hydrox
