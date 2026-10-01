// Copyright (c) 2026 OceanX
#pragma once
#include "hydrox/default_parameter.h"

#include "gnc/control_interfaces.h"

namespace hydrox
{
    /** Position/attitude controller for lift-plus-cruise VTOL aircraft. */
    class VtolController : public IController
    {
    public:
        struct Params
        {
            double mass = 5.0;
            double g = 9.80665;
            double z_kp = 1.8;
            double z_kd = 1.5;
            double roll_kp = 3.2;
            double roll_kd = 1.3;
            double pitch_kp = 3.2;
            double pitch_kd = 1.3;
            double yaw_kp = 1.5;
            double yaw_kd = 0.7;
            double max_roll_moment_Nm = 1.0;
            double max_pitch_moment_Nm = 1.0;
            double max_yaw_moment_Nm = 1.0;
            double xy_kp = 0.45;
            double xy_kd = 1.1;
            double max_tilt_rad = 0.35;
            double hover_max_speed_mps = 4.0;
            double hover_hold_max_speed_mps = 2.0;
            double cruise_speed_mps = 12.0;
            // Physical pusher-force model. Quadratic feed-forward balances
            // aerodynamic drag; proportional feedback closes the speed loop.
            double pusher_trim_force_N = 0.0;
            double pusher_drag_force_N_per_mps2 = 0.0507;
            double pusher_speed_kp_N_per_mps = 2.0;
            double max_pusher_force_N = 44.0;
            double transition_command_min_speed_mps = 8.0;
            double transition_start_mps = 8.0;
            double transition_end_mps = 14.0;
            double backtransition_complete_mps = 3.0;
            double cruise_course_kp = 1.0;
            double cruise_roll_limit_rad = 0.35;
            // Speed-scaled L1 path guidance for wing-borne flight.
            double cruise_l1_period_s = 16.0;
            double cruise_l1_damping = 0.80;
            double cruise_l1_min_distance_m = 24.0;
            double cruise_l1_max_course_error_rad = 1.3962634016;
            double max_course_reference_rate_radps = 0.35;
            double cruise_altitude_kp = 0.025;
            double cruise_altitude_kd = 0.12;
            double cruise_pitch_limit_rad = 0.22;
            double pusher_heading_gate_rad = 1.0471975512;
            double pusher_speed_ramp_mps2 = 2.0;
        };

        enum class FlightMode
        {
            Hover,
            FrontTransition,
            Cruise,
            BackTransition
        };

        explicit VtolController(const Params& p = detail::default_parameter<Params>());
        void reset(const NavigationState& state) override;
        void set_mode(GNCMode mode) override;
        void set_setpoint(const GNCSetpoint& sp) override { _sp = sp; }
        Wrench update(const NavigationState& state, double dt) override;

        FlightMode flight_mode() const { return _flight_mode; }
        bool transition_fault() const noexcept override { return _transition_fault; }
        bool acknowledge_transition_fault() noexcept override
        {
            if (_flight_mode != FlightMode::Hover) return false;
            _transition_fault = false;
            return true;
        }
        FlightPhase flight_phase() const noexcept override
        {
            switch (_flight_mode)
            {
            case FlightMode::Hover: return FlightPhase::Hover;
            case FlightMode::FrontTransition: return FlightPhase::FrontTransition;
            case FlightMode::Cruise: return FlightPhase::Cruise;
            case FlightMode::BackTransition: return FlightPhase::BackTransition;
            }
            return FlightPhase::NotApplicable;
        }
    private:
        Params _p;
        void enter_flight_mode(FlightMode mode, const NavigationState& state);

        GNCMode _mode = GNCMode::DISABLED;
        GNCSetpoint _sp;
        double _heading_hold = 0.0;
        FlightMode _flight_mode = FlightMode::Hover;
        double _flight_mode_elapsed_s = 0.0;
        double _pusher_speed_reference_mps = 0.0;
        double _course_reference_rad = 0.0;
        bool _course_reference_valid = false;
        bool _heading_hold_valid = false;
        bool _transition_fault = false;
        double _transition_entry_depth_m = 0.0;
    };
}
