// Copyright (c) 2026 OceanX. Author: xuheda
#pragma once
#include "hydrox/default_parameter.h"

#include "gnc/control_interfaces.h"

namespace hydrox
{
    class MultirotorController : public IController
    {
    public:
        struct Params
        {
            double mass = 1.375;
            double g = 9.80665;
            // Principal body inertias. The attitude loop computes angular
            // acceleration; multiply by inertia here so its wrench remains in
            // physical N*m all the way into the allocator.
            double Ixx = 0.0238405;
            // Imported rotor-model coefficients used to cancel the source
            // motor plugin's velocity-proportional rolling moment.
            double rotor_thrust_coefficient = 0.0;
            double rotor_rolling_moment_coefficient = 0.0;
            double Iyy = 0.0238405;
            double Izz = 0.043894;
            double z_kp = 2.0;
            double z_ki = 0.0;
            double z_integral_accel_limit = 0.0;
            double z_kd = 1.6;
            double roll_kp = 4.0;
            double roll_kd = 1.6;
            double pitch_kp = 4.0;
            double pitch_kd = 1.6;
            double yaw_kp = 1.8;
            double yaw_kd = 0.8;
            double xy_kp = 0.6;
            // Compatibility names: xy_kp is position P and xy_kd is velocity P.
            double xy_kd = 1.4;
            double xy_velocity_ki = 0.0;
            double xy_accel_damping = 0.0;
            double xy_integral_accel_limit = 0.0;
            double xy_anti_windup_gain = 1.0;
            // A zero mission speed selects this explicit hover-recapture
            // limit. It replaces the old hidden 0.5 m/s fallback.
            double hold_max_speed_mps = 0.35;
            // Reference dynamics. Position feedback creates a bounded target
            // velocity; these limits make velocity and acceleration continuous.
            double velocity_reference_tau_s = 0.30;
            double max_xy_jerk_mps3 = 2.0;
            // Damping is applied to a filtered derivative of NED velocity.
            double measured_accel_filter_tau_s = 0.25;
            double max_tilt_rad = 0.45;
            double max_xy_accel = 3.0;
            double max_xy_brake_accel = 3.0;
            double max_roll_accel_radps2 = 6.0;
            double max_pitch_accel_radps2 = 6.0;
            double max_yaw_accel_radps2 = 4.0;
            double max_z_accel = 5.0;
        };

        explicit MultirotorController(const Params &p = detail::default_parameter<Params>());

        void reset(const NavigationState &state) override;
        void set_mode(GNCMode mode) override;
        void set_setpoint(const GNCSetpoint &sp) override { _sp = sp; }
        Wrench update(const NavigationState &state, double dt) override;

    private:
        Params _p;
        GNCMode _mode = GNCMode::DISABLED;
        GNCSetpoint _sp;
        double _heading_hold = 0.0;
        bool _heading_hold_valid = false;
        bool _translation_initialized = false;
        Eigen::Vector2d _velocity_ref_ned = Eigen::Vector2d::Zero();
        Eigen::Vector2d _acceleration_ref_ned = Eigen::Vector2d::Zero();
        Eigen::Vector2d _velocity_error_integral = Eigen::Vector2d::Zero();
        Eigen::Vector2d _last_velocity_ned = Eigen::Vector2d::Zero();
        Eigen::Vector2d _filtered_accel_ned = Eigen::Vector2d::Zero();
        double _z_error_integral = 0.0;
    };
} // namespace hydrox
