// Copyright (c) 2026 OceanX. Author: xuheda
#include "gnc/surface_guidance.h"

#include <algorithm>
#include <cmath>

namespace hydrox
{
    namespace
    {
        constexpr double kPi = 3.14159265358979323846;

        double clamp_abs(double value, double limit)
        {
            return std::max(-limit, std::min(limit, value));
        }

        double wrap_pi(double angle)
        {
            while (angle > kPi)
                angle -= 2.0 * kPi;
            while (angle < -kPi)
                angle += 2.0 * kPi;
            return angle;
        }
    } // namespace

    SurfaceVesselGuidance::SurfaceVesselGuidance(const Params &p) : _p(p) {}

    void SurfaceVesselGuidance::reset(const NavigationState &state)
    {
        _initialized = true;
        _segment_active = false;
        _station_reacquiring = false;
        _cross_track_integral = 0.0;
        _last_surge_ref_mps = std::max(0.0, state.nu[0]);
        const double effective_surge = std::max(0.15, std::abs(state.nu[0]));
        _filtered_sideslip_rad = std::atan2(state.nu[1], effective_surge);
    }

    void SurfaceVesselGuidance::clear()
    {
        _initialized = false;
        _segment_active = false;
        _station_reacquiring = false;
        _cross_track_integral = 0.0;
        _filtered_sideslip_rad = 0.0;
        _last_surge_ref_mps = 0.0;
    }

    SurfaceGuidanceCommand SurfaceVesselGuidance::update(
        const NavigationState &state,
        const GNCSetpoint &setpoint,
        GNCMode mode,
        double dt)
    {
        SurfaceGuidanceCommand command;
        command.heading_ref_rad = setpoint.heading_ref;
        command.course_ref_rad = setpoint.heading_ref;
        command.surge_ref_mps = setpoint.surge_ref;
        command.unshaped_surge_ref_mps = setpoint.surge_ref;

        if (mode != GNCMode::WAYPOINT_3D)
        {
            _segment_active = false;
            _cross_track_integral = 0.0;
            _last_surge_ref_mps = setpoint.surge_ref;
            return command;
        }

        if (!_initialized)
            reset(state);
        const double control_dt = std::max(0.0, std::min(dt, 0.2));

        const double delta_n = setpoint.wp_n - state.eta[0];
        const double delta_e = setpoint.wp_e - state.eta[1];
        command.waypoint_distance_m = std::sqrt(
            delta_n * delta_n + delta_e * delta_e);

        const double requested_speed = std::min(
            std::max(0.0, setpoint.surge_ref),
            std::max(0.0, _p.waypoint_surge_mps));
        const bool terminal_heading = setpoint.hold_heading ||
                                      requested_speed <= 0.01;
        const double arrival_radius = std::max(0.0, setpoint.arrival_radius_m);
        const bool station_keep_enabled =
            setpoint.hold_heading && _p.station_keep_surge_mps > 0.01;
        const double station_keep_hysteresis = std::max(
            0.0, _p.station_keep_hysteresis_m);
        if (!station_keep_enabled)
        {
            _station_reacquiring = false;
        }
        else if (_station_reacquiring)
        {
            // Exit at the middle of the quiet band. Requiring a twin-screw
            // vessel to reach the original action radius under waves keeps
            // propulsion active too long and creates a limit cycle.
            if (command.waypoint_distance_m <=
                arrival_radius + 0.5 * station_keep_hysteresis)
                _station_reacquiring = false;
        }
        else if (command.waypoint_distance_m >
                 arrival_radius + station_keep_hysteresis)
        {
            _station_reacquiring = true;
        }
        // The outer trigger and inner release radius form a genuine
        // Schmitt band; small wave excursions no longer chatter propulsion.
        const bool station_reacquire =
            station_keep_enabled && _station_reacquiring;
        const bool track_waypoint = !terminal_heading || station_reacquire;
        const double guidance_speed = station_reacquire
            ? std::min(std::max(0.0, _p.station_keep_surge_mps),
                       std::max(0.0, _p.waypoint_surge_mps))
            : requested_speed;

        double desired_course = setpoint.heading_ref;
        if (!terminal_heading && setpoint.use_path_segment)
        {
            const double segment_n = setpoint.wp_n - setpoint.path_start_n;
            const double segment_e = setpoint.wp_e - setpoint.path_start_e;
            const double segment_length = std::hypot(segment_n, segment_e);
            if (segment_length > 0.25)
            {
                const bool changed = !_segment_active ||
                    std::abs(_segment_start_n - setpoint.path_start_n) > 1.0e-6 ||
                    std::abs(_segment_start_e - setpoint.path_start_e) > 1.0e-6 ||
                    std::abs(_segment_goal_n - setpoint.wp_n) > 1.0e-6 ||
                    std::abs(_segment_goal_e - setpoint.wp_e) > 1.0e-6;
                if (changed)
                {
                    _cross_track_integral = 0.0;
                    _segment_start_n = setpoint.path_start_n;
                    _segment_start_e = setpoint.path_start_e;
                    _segment_goal_n = setpoint.wp_n;
                    _segment_goal_e = setpoint.wp_e;
                }
                _segment_active = true;

                const double path_heading = std::atan2(segment_e, segment_n);
                const double rel_n = state.eta[0] - setpoint.path_start_n;
                const double rel_e = state.eta[1] - setpoint.path_start_e;
                command.cross_track_error_m =
                    -std::sin(path_heading) * rel_n +
                    std::cos(path_heading) * rel_e;
                const double lookahead = std::max(
                    1.0,
                    setpoint.lookahead_m > 0.0
                        ? setpoint.lookahead_m
                        : _p.los_lookahead_m);
                if (std::abs(command.cross_track_error_m) < 3.0 * lookahead &&
                    requested_speed > 0.1)
                {
                    _cross_track_integral +=
                        command.cross_track_error_m * control_dt;
                    _cross_track_integral = clamp_abs(
                        _cross_track_integral,
                        std::max(0.0, _p.ilos_integral_limit));
                }
                const double integral_offset =
                    _p.ilos_integral_gain * _cross_track_integral;
                desired_course = wrap_pi(
                    path_heading - std::atan2(
                        command.cross_track_error_m + integral_offset,
                        lookahead));
                // LOS tracks a finite leg, not its infinite extension. A wave
                // can carry the vessel past the finish plane outside the
                // arrival sphere; point pursuit must then recover the missed
                // waypoint instead of driving farther away along the old leg.
                const double along_track =
                    std::cos(path_heading) * rel_n +
                    std::sin(path_heading) * rel_e;
                if (along_track >= segment_length)
                {
                    desired_course = std::atan2(delta_e, delta_n);
                    _cross_track_integral = 0.0;
                }
            }
            else
            {
                _segment_active = false;
            }
        }
        else
        {
            _segment_active = false;
            _cross_track_integral = 0.0;
        }

        if (track_waypoint && !_segment_active &&
            command.waypoint_distance_m > 0.2)
        {
            desired_course = std::atan2(delta_e, delta_n);
        }
        command.course_ref_rad = wrap_pi(desired_course);

        // Crab into measured body sway so the ground track, not merely the
        // bow, follows the waypoint line. Low-pass filtering rejects the
        // wave-frequency sway that should be handled by passive hull motion.
        const double effective_surge = std::max(0.15, std::abs(state.nu[0]));
        const double sideslip = std::atan2(state.nu[1], effective_surge);
        const double beta_tau = std::max(
            1.0e-3, _p.sideslip_filter_time_constant_s);
        const double beta_alpha = 1.0 - std::exp(-control_dt / beta_tau);
        _filtered_sideslip_rad +=
            (sideslip - _filtered_sideslip_rad) * beta_alpha;
        const double crab = clamp_abs(
            _p.sideslip_compensation_gain * _filtered_sideslip_rad,
            std::max(0.0, _p.max_crab_angle_rad));
        command.heading_ref_rad = track_waypoint
            ? wrap_pi(command.course_ref_rad - crab)
            : wrap_pi(setpoint.heading_ref);

        // Braking-distance planning makes arrival independent of control
        // frequency and ship mass. The downstream speed loop then performs
        // active braking instead of relying on an unbounded coast-through.
        const double distance_to_brake = std::max(
            0.0, command.waypoint_distance_m - arrival_radius);
        const double braking_speed = std::sqrt(
            2.0 * std::max(0.05, _p.max_decel_mps2) * distance_to_brake);
        double shaped_speed = track_waypoint
            ? std::min(guidance_speed, braking_speed)
            : 0.0;

        // A twin-screw USV is underactuated. Turn toward the line of sight
        // before spending authority on forward motion.
        const double line_of_sight_error = wrap_pi(
            command.heading_ref_rad - state.eta[5]);
        shaped_speed *= std::max(0.0, std::cos(line_of_sight_error));
        command.unshaped_surge_ref_mps = shaped_speed;

        // Never keep a positive shaped reference merely because the previous
        // leg was faster than the vessel's actual speed. This preserves a
        // smooth stop for a coasting hull while commanding zero immediately
        // when the vessel is already stationary at the terminal point.
        if (terminal_heading && !station_reacquire)
            _last_surge_ref_mps = std::min(_last_surge_ref_mps, std::max(0.0, state.nu[0]));
        const double delta_speed = shaped_speed - _last_surge_ref_mps;
        const double rate = delta_speed >= 0.0
            ? std::max(0.0, _p.max_accel_mps2)
            : std::max(0.0, _p.max_decel_mps2);
        const double max_step = rate * control_dt;
        _last_surge_ref_mps += clamp_abs(delta_speed, max_step);
        if (std::abs(_last_surge_ref_mps) < 1.0e-6)
            _last_surge_ref_mps = 0.0;
        command.surge_ref_mps = std::max(0.0, _last_surge_ref_mps);
        return command;
    }
} // namespace hydrox
