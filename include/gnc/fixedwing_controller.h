// Copyright (c) 2026 OceanX. Author: xuheda
#pragma once
#include "hydrox/default_parameter.h"

#include "gnc/control_interfaces.h"

namespace hydrox
{
    class FixedWingController : public IController
    {
    public:
        struct Params
        {
            double cruise_speed_mps = 12.0;
            // The controller owns the speed loop and emits physical surge force.
            double cruise_force_N = 4.32;
            double speed_force_kp_N_per_mps = 0.144;
            double max_forward_force_N = 12.0;
            // Rigid-body inertia converts attitude-loop angular acceleration
            // demands into physical body moments.
            double Ixx_kgm2 = 0.201497;
            double Iyy_kgm2 = 0.149361;
            double Izz_kgm2 = 0.150727;
            double yaw_rate_kp_radps2_per_radps = 1.0;
            double max_yaw_accel_radps2 = 0.50;
            double altitude_kp = 0.08;
            double altitude_kd = 0.05;
            double altitude_ki = 0.0;
            double altitude_integral_limit = 10.0;
            // The planner supplies a continuously moving NED depth reference.
            // Convert its rate into a flight-path-angle feed-forward instead
            // of waiting for altitude error to build up.
            double depth_reference_rate_filter_tau_s = 0.8;
            double depth_reference_rate_stale_s = 0.5;
            double max_depth_reference_rate_mps = 2.0;
            double flight_path_feedforward_gain = 1.0;
            double pitch_limit_rad = 0.35;
            double max_pitch_reference_rate_radps = 0.18;
            // Airspeed is not yet a separately observed state, so forward
            // body speed is the conservative protection signal. Altitude is
            // sacrificed before the controller is allowed to demand a
            // stall-inducing climb.
            double min_flight_speed_mps = 9.0;
            double max_flight_speed_mps = 16.0;
            double underspeed_pitch_gain = 0.08;
            double overspeed_pitch_gain = 0.025;

            // L1 lateral guidance. The look-ahead distance scales with ground
            // speed, giving comparable damping on straight legs and turns.
            double l1_period_s = 22.0;
            double l1_damping = 0.85;
            double l1_min_distance_m = 20.0;
            double l1_max_course_error_rad = 1.3962634016;
            double course_kp = 0.9;
            double roll_limit_rad = 0.55;
            double max_roll_reference_rate_radps = 0.40;
            double max_coordinated_yaw_rate_radps = 0.50;

            // Inner attitude loops convert the outer-loop references into
            // bounded actuator demands.  Without these rate-damped loops a
            // waypoint reference is effectively held as a full control
            // surface deflection and the small fixed wing departs rapidly.
            double roll_attitude_kp = 0.65;
            double roll_rate_kd = 0.28;
            double max_roll_accel_radps2 = 0.16;
            double pitch_attitude_kp = 0.70;
            double pitch_attitude_ki = 0.18;
            double pitch_error_integral_limit = 0.60;
            double pitch_rate_kd = 0.38;
            // Physical feed-forward moment balancing the trimmed airframe at
            // the reference operating point.
            double pitch_trim_moment_Nm = 0.02389776;
            double max_pitch_accel_radps2 = 0.40;
        };

        explicit FixedWingController(const Params &p = detail::default_parameter<Params>());

        void reset(const NavigationState &state) override;
        void set_mode(GNCMode mode) override;
        void set_setpoint(const GNCSetpoint &sp) override { _sp = sp; }
        Wrench update(const NavigationState &state, double dt) override;

    private:
        Params _p;
        GNCMode _mode = GNCMode::DISABLED;
        GNCSetpoint _sp;
        double _altitude_error_integral = 0.0;
        double _pitch_error_integral = 0.0;
        double _roll_reference_rad = 0.0;
        double _pitch_reference_rad = 0.0;
        double _last_depth_reference_m = 0.0;
        double _depth_reference_elapsed_s = 0.0;
        double _depth_reference_rate_observation_mps = 0.0;
        double _filtered_depth_reference_rate_mps = 0.0;
        bool _references_initialized = false;
        bool _depth_reference_initialized = false;
    };
} // namespace hydrox
