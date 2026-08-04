#include "fossen_vehicle_params.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <vector>

namespace hydrox
{
namespace
{
    std::string compact_key(const std::string &s)
    {
        std::string out;
        for (unsigned char ch : s)
            if (std::isalnum(ch))
                out.push_back(static_cast<char>(std::tolower(ch)));
        return out;
    }

    bool read_text_file(const std::filesystem::path &path, std::string &out)
    {
        std::ifstream f(path, std::ios::binary);
        if (!f)
            return false;
        std::ostringstream ss;
        ss << f.rdbuf();
        out = ss.str();
        return true;
    }

    std::optional<size_t> find_key_value_pos(const std::string &json,
                                             const std::string &key)
    {
        const std::string needle = "\"" + key + "\"";
        size_t pos = 0;
        while ((pos = json.find(needle, pos)) != std::string::npos)
        {
            size_t i = pos + needle.size();
            while (i < json.size() && std::isspace(static_cast<unsigned char>(json[i])))
                ++i;
            if (i < json.size() && json[i] == ':')
                return i + 1;
            pos += needle.size();
        }
        return std::nullopt;
    }

    std::optional<std::string> extract_bracketed(const std::string &json,
                                                 size_t value_pos,
                                                 char open_ch,
                                                 char close_ch)
    {
        size_t i = value_pos;
        while (i < json.size() && std::isspace(static_cast<unsigned char>(json[i])))
            ++i;
        if (i >= json.size() || json[i] != open_ch)
            return std::nullopt;

        const size_t start = i;
        int depth = 0;
        bool in_string = false;
        bool escape = false;
        for (; i < json.size(); ++i)
        {
            const char ch = json[i];
            if (in_string)
            {
                if (escape)
                    escape = false;
                else if (ch == '\\')
                    escape = true;
                else if (ch == '"')
                    in_string = false;
                continue;
            }
            if (ch == '"')
                in_string = true;
            else if (ch == open_ch)
                ++depth;
            else if (ch == close_ch && --depth == 0)
                return json.substr(start, i - start + 1);
        }
        return std::nullopt;
    }

    std::optional<std::string> object_for_key(const std::string &json,
                                              const std::string &key)
    {
        const auto pos = find_key_value_pos(json, key);
        if (!pos)
            return std::nullopt;
        return extract_bracketed(json, *pos, '{', '}');
    }

    std::optional<std::string> array_for_key(const std::string &json,
                                             const std::string &key)
    {
        const auto pos = find_key_value_pos(json, key);
        if (!pos)
            return std::nullopt;
        return extract_bracketed(json, *pos, '[', ']');
    }

    std::optional<double> number_for_key(const std::string &json,
                                         const std::string &key)
    {
        const auto pos = find_key_value_pos(json, key);
        if (!pos)
            return std::nullopt;
        size_t i = *pos;
        while (i < json.size() && std::isspace(static_cast<unsigned char>(json[i])))
            ++i;
        char *end = nullptr;
        const double value = std::strtod(json.c_str() + i, &end);
        if (end == json.c_str() + i)
            return std::nullopt;
        return value;
    }

    std::vector<double> numbers_in_array(const std::string &array_text)
    {
        std::vector<double> out;
        const char *p = array_text.c_str();
        while (*p)
        {
            if (*p == '-' || *p == '+' || *p == '.' ||
                std::isdigit(static_cast<unsigned char>(*p)))
            {
                char *end = nullptr;
                const double value = std::strtod(p, &end);
                if (end != p)
                {
                    out.push_back(value);
                    p = end;
                    continue;
                }
            }
            ++p;
        }
        return out;
    }

    double max_abs_fin_x(const std::string &fins_object, double fallback)
    {
        const auto arr = array_for_key(fins_object, "fin_cop");
        if (!arr)
            return fallback;
        const auto vals = numbers_in_array(*arr);
        double x = 0.0;
        for (size_t i = 0; i + 2 < vals.size(); i += 3)
            x = std::max(x, std::abs(vals[i]));
        return x > 1e-6 ? x : fallback;
    }

    struct SlenderInertia
    {
        double M44_pitch = 0.0;
        double mass_total = 0.0;
    };

    SlenderInertia compute_slender_inertia(double mass, double L, double diam)
    {
        const double a = 0.5 * L;
        const double b = 0.5 * diam;
        const double e2 = std::max(1.0 - (b * b) / (a * a), 1e-12);
        const double e = std::sqrt(e2);
        const double log_ep = std::log((1.0 + e) / (1.0 - e));
        const double alpha0 = (2.0 * (1.0 - e2) / (e * e * e)) * (0.5 * log_ep - e);
        const double beta0 = 1.0 / e2 - (1.0 - e2) / (2.0 * e * e * e) * log_ep;
        const double k1 = alpha0 / (2.0 - alpha0);
        const double k_prime = e2 * e2 * (beta0 - alpha0) /
                               ((2.0 - e2) * (2.0 * e2 - (2.0 - e2) * (beta0 - alpha0)));
        const double Iy = 0.2 * mass * (a * a + b * b);
        return {Iy * (1.0 + k_prime), mass * (1.0 + k1)};
    }

    void set_error(std::string *error, const std::string &msg)
    {
        if (error)
            *error = msg;
    }

    void apply_depth_overrides(const std::string &obj, DepthPID::Params &p)
    {
        if (auto v = number_for_key(obj, "kp")) p.kp = *v;
        if (auto v = number_for_key(obj, "kd")) p.kd = *v;
        if (auto v = number_for_key(obj, "ki")) p.ki = *v;
        if (auto v = number_for_key(obj, "wn")) p.wn = *v;
        if (auto v = number_for_key(obj, "zeta")) p.zeta = *v;
        if (auto v = number_for_key(obj, "vmax")) p.vmax = *v;
        if (auto v = number_for_key(obj, "ei_max")) p.ei_max = *v;
        if (auto v = number_for_key(obj, "theta_max")) p.theta_max = *v;
    }

    void apply_pitch_overrides(const std::string &obj, PitchPID::Params &p)
    {
        if (auto v = number_for_key(obj, "kp")) p.kp = *v;
        if (auto v = number_for_key(obj, "kd")) p.kd = *v;
        if (auto v = number_for_key(obj, "ki")) p.ki = *v;
        if (auto v = number_for_key(obj, "ei_max")) p.ei_max = *v;
        if (auto v = number_for_key(obj, "tau_max")) p.tau_max = *v;
    }

    void apply_heading_overrides(const std::string &obj, HeadingSMC::Params &p)
    {
        if (auto v = number_for_key(obj, "T")) p.T = *v;
        if (auto v = number_for_key(obj, "K")) p.K = *v;
        if (auto v = number_for_key(obj, "wn")) p.wn = *v;
        if (auto v = number_for_key(obj, "zeta")) p.zeta = *v;
        if (auto v = number_for_key(obj, "vmax")) p.vmax = *v;
        if (auto v = number_for_key(obj, "kd")) p.kd = *v;
        if (auto v = number_for_key(obj, "ks")) p.ks = *v;
        if (auto v = number_for_key(obj, "phi_b")) p.phi_b = *v;
        if (auto v = number_for_key(obj, "ei_max")) p.ei_max = *v;
    }

    void apply_yaw_rate_overrides(const std::string &obj,
                                  SlenderBodyAUVController::Params::YawRatePI &p)
    {
        if (auto v = number_for_key(obj, "kp")) p.kp = *v;
        if (auto v = number_for_key(obj, "ki")) p.ki = *v;
        if (auto v = number_for_key(obj, "kd")) p.kd = *v;
        if (auto v = number_for_key(obj, "ei_max")) p.ei_max = *v;
        if (auto v = number_for_key(obj, "tau_max")) p.tau_max = *v;
        if (auto v = number_for_key(obj, "command_gain")) p.command_gain = *v;
        if (auto v = number_for_key(obj, "feed_forward")) p.feed_forward = *v;
        if (auto v = number_for_key(obj, "ref_filter_tau")) p.ref_filter_tau = *v;
        if (auto v = number_for_key(obj, "ref_slew_limit")) p.ref_slew_limit = *v;
    }

    void apply_surge_overrides(const std::string &obj, SurgeP::Params &p)
    {
        if (auto v = number_for_key(obj, "kp")) p.kp = *v;
        if (auto v = number_for_key(obj, "ki")) p.ki = *v;
        if (auto v = number_for_key(obj, "kd")) p.kd = *v;
        if (auto v = number_for_key(obj, "integral_limit")) p.integral_limit = *v;
        if (auto v = number_for_key(obj, "unwind_gain")) p.unwind_gain = *v;
        if (auto v = number_for_key(obj, "drag_ff")) p.drag_ff = *v;
        if (auto v = number_for_key(obj, "forward_min_tau")) p.forward_min_tau = *v;
        if (auto v = number_for_key(obj, "tau_rate_limit")) p.tau_rate_limit = *v;
        if (auto v = number_for_key(obj, "overspeed_deadband")) p.overspeed_deadband = *v;
        if (auto v = number_for_key(obj, "overspeed_brake_gain")) p.overspeed_brake_gain = *v;
        if (auto v = number_for_key(obj, "overspeed_brake_tau_max")) p.overspeed_brake_tau_max = *v;
    }

    void apply_turn_speed_overrides(
        const std::string &obj,
        SlenderBodyAUVController::Params::TurnSpeedCompensation &p)
    {
        if (auto v = number_for_key(obj, "turn_speed_drop_start_radps")) p.drop_start_radps = *v;
        if (auto v = number_for_key(obj, "turn_speed_drop_gain_mps_per_radps")) p.drop_gain_mps_per_radps = *v;
        if (auto v = number_for_key(obj, "turn_speed_drop_max_mps")) p.drop_max_mps = *v;
    }

    void apply_thruster_axis_overrides(const std::string &obj,
                                       ThrusterVehicleController::Axis &p)
    {
        if (auto v = number_for_key(obj, "kp")) p.kp = *v;
        if (auto v = number_for_key(obj, "kd")) p.kd = *v;
        if (auto v = number_for_key(obj, "accel_max")) p.accel_max = *v;
    }

    void apply_thruster_control_overrides(const std::string &control,
                                          FossenControlParams &params)
    {
        const auto tuning = object_for_key(control, "thruster");
        if (!tuning)
            return;
        if (const auto axis = object_for_key(*tuning, "surge")) apply_thruster_axis_overrides(*axis, params.thruster_gnc.surge);
        if (const auto axis = object_for_key(*tuning, "sway")) apply_thruster_axis_overrides(*axis, params.thruster_gnc.sway);
        if (const auto axis = object_for_key(*tuning, "heave")) apply_thruster_axis_overrides(*axis, params.thruster_gnc.heave);
        if (const auto axis = object_for_key(*tuning, "roll")) apply_thruster_axis_overrides(*axis, params.thruster_gnc.roll);
        if (const auto axis = object_for_key(*tuning, "pitch")) apply_thruster_axis_overrides(*axis, params.thruster_gnc.pitch);
        if (const auto axis = object_for_key(*tuning, "yaw")) apply_thruster_axis_overrides(*axis, params.thruster_gnc.yaw);
        params.archetype_control_loaded_from_json = true;
    }

    void apply_surface_control_overrides(const std::string &control,
                                         FossenControlParams &params)
    {
        const auto tuning = object_for_key(control, "surface");
        if (!tuning)
            return;
        auto &p = params.surface_gnc;
        if (auto v = number_for_key(*tuning, "surge_kp")) p.surge_kp = *v;
        if (auto v = number_for_key(*tuning, "surge_kd")) p.surge_kd = *v;
        if (auto v = number_for_key(*tuning, "surge_ki")) p.surge_ki = *v;
        if (auto v = number_for_key(*tuning, "surge_integral_limit")) p.surge_integral_limit = *v;
        if (auto v = number_for_key(*tuning, "yaw_kp")) p.yaw_kp = *v;
        if (auto v = number_for_key(*tuning, "yaw_kd")) p.yaw_kd = *v;
        if (auto v = number_for_key(*tuning, "max_force_N")) p.max_force_N = *v;
        if (auto v = number_for_key(*tuning, "max_moment_Nm")) p.max_moment_Nm = *v;
        if (auto v = number_for_key(*tuning, "waypoint_surge_mps")) p.waypoint_surge_mps = *v;
        if (auto v = number_for_key(*tuning, "sideslip_compensation_gain")) p.sideslip_compensation_gain = *v;
        if (auto v = number_for_key(*tuning, "max_crab_angle_rad")) p.max_crab_angle_rad = *v;
        params.archetype_control_loaded_from_json = true;
    }

    void apply_multirotor_control_overrides(const std::string &control,
                                            FossenControlParams &params)
    {
        const auto tuning = object_for_key(control, "multirotor");
        if (!tuning)
            return;
        auto &p = params.multirotor_gnc;
        if (auto v = number_for_key(*tuning, "z_kp")) p.z_kp = *v;
        if (auto v = number_for_key(*tuning, "z_kd")) p.z_kd = *v;
        if (auto v = number_for_key(*tuning, "roll_kp")) p.roll_kp = *v;
        if (auto v = number_for_key(*tuning, "roll_kd")) p.roll_kd = *v;
        if (auto v = number_for_key(*tuning, "pitch_kp")) p.pitch_kp = *v;
        if (auto v = number_for_key(*tuning, "pitch_kd")) p.pitch_kd = *v;
        if (auto v = number_for_key(*tuning, "yaw_kp")) p.yaw_kp = *v;
        if (auto v = number_for_key(*tuning, "yaw_kd")) p.yaw_kd = *v;
        if (auto v = number_for_key(*tuning, "xy_kp")) p.xy_kp = *v;
        if (auto v = number_for_key(*tuning, "xy_kd")) p.xy_kd = *v;
        if (auto v = number_for_key(*tuning, "max_tilt_rad")) p.max_tilt_rad = *v;
        if (auto v = number_for_key(*tuning, "max_xy_accel")) p.max_xy_accel = *v;
        if (auto v = number_for_key(*tuning, "max_z_accel")) p.max_z_accel = *v;
        params.archetype_control_loaded_from_json = true;
    }

    void apply_fixedwing_control_overrides(const std::string &control,
                                           FossenControlParams &params)
    {
        const auto tuning = object_for_key(control, "fixedwing");
        if (!tuning)
            return;
        auto &p = params.fixedwing_gnc;
        if (auto v = number_for_key(*tuning, "cruise_speed_mps")) p.cruise_speed_mps = *v;
        if (auto v = number_for_key(*tuning, "altitude_kp")) p.altitude_kp = *v;
        if (auto v = number_for_key(*tuning, "altitude_kd")) p.altitude_kd = *v;
        if (auto v = number_for_key(*tuning, "altitude_ki")) p.altitude_ki = *v;
        if (auto v = number_for_key(*tuning, "altitude_integral_limit")) p.altitude_integral_limit = *v;
        if (auto v = number_for_key(*tuning, "pitch_limit_rad")) p.pitch_limit_rad = *v;
        if (auto v = number_for_key(*tuning, "course_kp")) p.course_kp = *v;
        if (auto v = number_for_key(*tuning, "roll_limit_rad")) p.roll_limit_rad = *v;
        if (auto v = number_for_key(*tuning, "roll_attitude_kp")) p.roll_attitude_kp = *v;
        if (auto v = number_for_key(*tuning, "roll_rate_kd")) p.roll_rate_kd = *v;
        if (auto v = number_for_key(*tuning, "max_roll_command")) p.max_roll_command = *v;
        if (auto v = number_for_key(*tuning, "pitch_attitude_kp")) p.pitch_attitude_kp = *v;
        if (auto v = number_for_key(*tuning, "pitch_rate_kd")) p.pitch_rate_kd = *v;
        if (auto v = number_for_key(*tuning, "pitch_trim")) p.pitch_trim = *v;
        if (auto v = number_for_key(*tuning, "max_pitch_command")) p.max_pitch_command = *v;
        auto &allocator = params.fixedwing_allocator;
        if (auto v = number_for_key(*tuning, "elevator_kp")) allocator.elevator_kp = *v;
        if (auto v = number_for_key(*tuning, "aileron_kp")) allocator.aileron_kp = *v;
        if (auto v = number_for_key(*tuning, "rudder_kp")) allocator.rudder_kp = *v;
        if (auto v = number_for_key(*tuning, "throttle_trim")) allocator.throttle_trim = *v;
        if (auto v = number_for_key(*tuning, "throttle_kp")) allocator.throttle_kp = *v;
        params.archetype_control_loaded_from_json = true;
    }

    void apply_vtol_control_overrides(const std::string &control,
                                      FossenControlParams &params)
    {
        const auto tuning = object_for_key(control, "vtol");
        if (!tuning)
            return;
        auto &p = params.vtol_gnc;
        if (auto v = number_for_key(*tuning, "z_kp")) p.z_kp = *v;
        if (auto v = number_for_key(*tuning, "z_kd")) p.z_kd = *v;
        if (auto v = number_for_key(*tuning, "roll_kp")) p.roll_kp = *v;
        if (auto v = number_for_key(*tuning, "roll_kd")) p.roll_kd = *v;
        if (auto v = number_for_key(*tuning, "pitch_kp")) p.pitch_kp = *v;
        if (auto v = number_for_key(*tuning, "pitch_kd")) p.pitch_kd = *v;
        if (auto v = number_for_key(*tuning, "yaw_kp")) p.yaw_kp = *v;
        if (auto v = number_for_key(*tuning, "yaw_kd")) p.yaw_kd = *v;
        if (auto v = number_for_key(*tuning, "xy_kp")) p.xy_kp = *v;
        if (auto v = number_for_key(*tuning, "xy_kd")) p.xy_kd = *v;
        if (auto v = number_for_key(*tuning, "max_tilt_rad")) p.max_tilt_rad = *v;
        if (auto v = number_for_key(*tuning, "cruise_speed_mps")) p.cruise_speed_mps = *v;
        params.archetype_control_loaded_from_json = true;
    }

    void apply_gnc_overrides(const std::string &json,
                             FossenControlParams &params)
    {
        const auto control = object_for_key(json, "control");
        if (!control)
            return;
        if (const auto depth = object_for_key(*control, "depth"))
            apply_depth_overrides(*depth, params.gnc.depth);
        if (const auto pitch = object_for_key(*control, "pitch"))
            apply_pitch_overrides(*pitch, params.gnc.pitch);
        if (const auto heading = object_for_key(*control, "heading"))
            apply_heading_overrides(*heading, params.gnc.heading);
        if (const auto yaw_rate = object_for_key(*control, "yaw_rate"))
        {
            apply_yaw_rate_overrides(*yaw_rate, params.gnc.yaw_rate);
            params.yaw_rate_control_loaded_from_json = true;
        }
        if (const auto surge = object_for_key(*control, "surge"))
        {
            apply_surge_overrides(*surge, params.gnc.surge);
            apply_turn_speed_overrides(*surge, params.gnc.turn_speed);
        }
    }

    bool validate(FossenControlParams &p, std::string *error)
    {
        if (p.archetype == VehicleArchetype::DifferentialDrive)
        {
            const auto &g = p.ground_allocator;
            if (p.mass_total <= 0.0 || g.wheel_radius_m <= 0.0 ||
                g.track_width_m <= 0.0 ||
                g.max_wheel_angular_speed_radps <= 0.0 ||
                g.longitudinal_speed_gain_N_per_mps <= 0.0)
            {
                set_error(error, "invalid differential-drive control parameters");
                p.valid = false;
                return false;
            }
            p.valid = true;
            return true;
        }
        if (p.archetype != VehicleArchetype::SlenderBodyFin)
        {
            if ((p.archetype == VehicleArchetype::Thruster ||
                 p.archetype == VehicleArchetype::Surface) &&
                p.max_thrust_per_thruster_N <= 0.0)
            {
                set_error(error, "invalid thruster max thrust");
                p.valid = false;
                return false;
            }
            p.valid = true;
            return true;
        }

        const auto &a = p.allocator;
        if (a.S_fin <= 0.0 || a.CL_s <= 0.0 || a.CL_r <= 0.0 ||
            a.x_fin <= 0.0 || a.D_prop <= 0.0 || a.n_max_rpm <= 0.0 ||
            a.max_thrust_N <= 0.0)
        {
            set_error(error, "invalid fin/propeller control parameters");
            p.valid = false;
            return false;
        }
        p.valid = true;
        return true;
    }

    std::vector<std::filesystem::path> candidate_paths(
        const std::string &filename,
        const std::string &explicit_path,
        const std::string &params_dir,
        VehicleArchetype archetype)
    {
        std::vector<std::filesystem::path> paths;
        (void)archetype;
        if (!explicit_path.empty())
            paths.emplace_back(explicit_path);
        if (!params_dir.empty())
            paths.emplace_back(std::filesystem::path(params_dir) / filename);
        return paths;
    }

    bool load_json_into(const std::filesystem::path &path,
                        FossenControlParams &params,
                        std::string *error)
    {
        std::string json;
        if (!read_text_file(path, json))
        {
            set_error(error, "failed to read " + path.string());
            return false;
        }

        const auto inertia = object_for_key(json, "inertia");
        const auto actuators = object_for_key(json, "actuators");
        const auto fins = actuators ? object_for_key(*actuators, "fins") : std::nullopt;
        const auto prop = actuators ? object_for_key(*actuators, "propeller") : std::nullopt;
        const auto control = object_for_key(json, "control");

        if (params.archetype == VehicleArchetype::DifferentialDrive)
        {
            const auto body = object_for_key(json, "body");
            const auto wheels = object_for_key(json, "wheels");
            const auto ground_motor = object_for_key(json, "motor");
            if (body)
                if (auto v = number_for_key(*body, "mass_kg")) params.mass_total = *v;
            if (wheels)
            {
                if (auto v = number_for_key(*wheels, "radius_m"))
                    params.ground_allocator.wheel_radius_m = *v;
                if (auto v = number_for_key(*wheels, "track_width_m"))
                    params.ground_allocator.track_width_m = *v;
            }
            if (ground_motor)
            {
                if (auto v = number_for_key(*ground_motor, "max_angular_speed_radps"))
                    params.ground_allocator.max_wheel_angular_speed_radps = *v;
                if (auto v = number_for_key(*ground_motor, "speed_gain_n_per_mps"))
                    params.ground_allocator.longitudinal_speed_gain_N_per_mps = *v;
            }
            params.source_path = path.string();
            params.loaded_from_json = true;
            return validate(params, error);
        }

        if (params.archetype == VehicleArchetype::Surface)
        {
            const auto propulsion = object_for_key(json, "propulsion");
            if (inertia)
            {
                const double hull = number_for_key(*inertia, "m_hull").value_or(0.0);
                const double payload = number_for_key(*inertia, "m_payload").value_or(0.0);
                if (hull + payload > 0.0)
                    params.mass_total = hull + payload;
            }
            if (propulsion)
            {
                if (auto v = number_for_key(*propulsion, "max_thrust_per_motor_N"))
                    params.max_thrust_per_thruster_N = *v;
                else if (auto v = number_for_key(*propulsion, "bollard_pos_N"))
                    params.max_thrust_per_thruster_N = *v;

                params.surface_channel_surge_limit_N = number_for_key(
                    *propulsion, "max_thrust_per_control_channel_N").value_or(
                    params.max_thrust_per_thruster_N);
                params.surface_channel_lever_arm_m = number_for_key(
                    *propulsion, "control_channel_lever_arm_m").value_or(0.395);
            }
            if (control)
                apply_surface_control_overrides(*control, params);
            params.source_path = path.string();
            params.loaded_from_json = true;
            return validate(params, error);
        }

        if (params.archetype == VehicleArchetype::Thruster)
        {
            if (inertia)
            {
                if (auto v = number_for_key(*inertia, "mass"))
                    params.mass_total = *v;
            }
            if (actuators)
            {
                if (auto v = number_for_key(*actuators, "max_thrust_per_thruster_N"))
                    params.max_thrust_per_thruster_N = *v;
            }
            if (control)
                apply_thruster_control_overrides(*control, params);
            params.source_path = path.string();
            params.loaded_from_json = true;
            return validate(params, error);
        }

        if (params.archetype == VehicleArchetype::Multirotor ||
            params.archetype == VehicleArchetype::FixedWing ||
            params.archetype == VehicleArchetype::VTOL)
        {
            std::optional<std::string> air_body = object_for_key(json, "body");
            std::optional<std::string> air_motor = object_for_key(json, "motor");
            if (params.archetype == VehicleArchetype::VTOL)
            {
                if (const auto lift = object_for_key(json, "lift"))
                {
                    air_body = object_for_key(*lift, "body");
                    air_motor = object_for_key(*lift, "motor");
                }
            }
            if (air_body)
            {
                if (auto v = number_for_key(*air_body, "mass"))
                    params.mass_total = *v;
            }
            if ((params.archetype == VehicleArchetype::Multirotor ||
                 params.archetype == VehicleArchetype::VTOL) && air_motor)
            {
                const double kT = number_for_key(*air_motor, "kT").value_or(0.0);
                const double omega_max = number_for_key(*air_motor, "omega_max").value_or(0.0);
                if (kT > 0.0 && omega_max > 0.0)
                    params.max_total_lift_N = 4.0 * kT * omega_max * omega_max;
            }
            if (control)
            {
                if (params.archetype == VehicleArchetype::Multirotor)
                    apply_multirotor_control_overrides(*control, params);
                else if (params.archetype == VehicleArchetype::FixedWing)
                    apply_fixedwing_control_overrides(*control, params);
                else
                    apply_vtol_control_overrides(*control, params);
            }
            params.source_path = path.string();
            params.loaded_from_json = true;
            return validate(params, error);
        }

        if (!fins || !prop)
        {
            set_error(error, "missing actuators.fins or actuators.propeller in " + path.string());
            return false;
        }

        auto &a = params.allocator;
        auto &m = params.motor;
        if (inertia)
        {
            if (auto rho = number_for_key(*inertia, "rho"))
            {
                a.rho = *rho;
                m.rho = *rho;
            }
        }

        if (auto v = number_for_key(*fins, "fin_area_m2")) a.S_fin = *v;
        if (auto v = number_for_key(*fins, "S_fin")) a.S_fin = *v;
        if (auto v = number_for_key(*fins, "allocator_CL_s")) a.CL_s = *v;
        if (auto v = number_for_key(*fins, "allocator_CL_r")) a.CL_r = *v;
        if (auto v = number_for_key(*fins, "fin_max_deflection_deg")) a.delta_max_deg = *v;
        if (auto v = number_for_key(*fins, "delta_max_deg")) a.delta_max_deg = *v;
        if (auto v = number_for_key(*fins, "allocator_u_min")) a.u_min = *v;
        if (auto v = number_for_key(*fins, "u_min")) a.u_min = *v;
        a.x_fin = max_abs_fin_x(*fins, a.x_fin);

        if (auto v = number_for_key(*prop, "max_thrust_N")) a.max_thrust_N = *v;
        if (auto v = number_for_key(*prop, "D_prop"))
        {
            a.D_prop = *v;
            m.D_prop = *v;
        }
        if (auto v = number_for_key(*prop, "KT_0"))
        {
            a.KT_0 = *v;
            m.KT_0 = *v;
        }
        if (auto v = number_for_key(*prop, "KQ_0")) m.KQ_0 = *v;
        if (auto v = number_for_key(*prop, "n_max_rpm"))
        {
            a.n_max_rpm = *v;
            m.rpm_max = *v;
        }
        if (auto v = number_for_key(*prop, "T_n")) m.tau_m = *v;
        apply_gnc_overrides(json, params);

        const auto geometry = object_for_key(json, "geometry");
        if (inertia && geometry)
        {
            const double mass = number_for_key(*inertia, "mass").value_or(0.0);
            const double L = number_for_key(*geometry, "length").value_or(0.0);
            const double diam = number_for_key(*geometry, "diameter").value_or(0.0);
            if (mass > 0.0 && L > 0.0 && diam > 0.0)
            {
                const auto si = compute_slender_inertia(mass, L, diam);
                params.M44_pitch = si.M44_pitch;
                params.mass_total = si.mass_total;
            }
        }

        params.source_path = path.string();
        params.loaded_from_json = true;
        return validate(params, error);
    }
} // namespace

std::string canonical_vehicle_type(const std::string &type)
{
    const std::string key = compact_key(type);
    if (key.empty() || key == "ecaa9") return "EcaA9";
    if (key == "ecaa9test") return "EcaA9Test";
    if (key == "lauv") return "LAUV";
    if (key == "lauvtest") return "LAUVTest";
    if (key == "desisteksaga") return "DesistekSaga";
    if (key == "rexrov2") return "RexROV2";
    if (key == "wamv" || key == "vrxwamv")
        return "WAMV";
    if (key == "surfacevessel" || key == "otter" || key == "otterusv" ||
        key == "usv")
        return "SurfaceVessel";
    if (key == "uav" || key == "quadrotor" || key == "multirotor" || key == "x500") return "X500";
    if (key == "fixedwing" || key == "aerosonde" || key == "rccessna") return "RCCessna";
    if (key == "standardvtol" || key == "vtol") return "StandardVTOL";
    if (key == "r1rover" || key == "rover" || key == "ugv" ||
        key == "differentialdrive") return "R1Rover";
    return type;
}

VehicleArchetype archetype_for(const std::string &type)
{
    const std::string t = canonical_vehicle_type(type);
    if (t == "DesistekSaga" || t == "RexROV2")
        return VehicleArchetype::Thruster;
    if (t == "SurfaceVessel" || t == "WAMV") return VehicleArchetype::Surface;
    if (t == "X500") return VehicleArchetype::Multirotor;
    if (t == "RCCessna") return VehicleArchetype::FixedWing;
    if (t == "StandardVTOL") return VehicleArchetype::VTOL;
    if (t == "R1Rover") return VehicleArchetype::DifferentialDrive;
    return VehicleArchetype::SlenderBodyFin;
}

std::string fossen_params_filename(const std::string &type)
{
    const std::string t = canonical_vehicle_type(type);
    if (t == "EcaA9") return "eca_a9_params.json";
    if (t == "LAUV") return "lauv_params.json";
    if (t == "EcaA9Test") return "eca_a9_test_params.json";
    if (t == "LAUVTest") return "lauv_test_params.json";
    if (t == "DesistekSaga") return "desistek_saga_params.json";
    if (t == "RexROV2") return "rexrov2_params.json";
    if (t == "SurfaceVessel") return "otter_params.json";
    if (t == "WAMV") return "wamv_params.json";
    if (t == "X500") return "x500_params.json";
    if (t == "RCCessna") return "rc_cessna_params.json";
    if (t == "StandardVTOL") return "standard_vtol_params.json";
    if (t == "R1Rover") return "r1_rover_params.json";
    return {};
}

FossenControlParams builtin_fossen_control_params(const std::string &type)
{
    FossenControlParams out;
    out.vehicle_type = canonical_vehicle_type(type);
    out.archetype = archetype_for(out.vehicle_type);
    out.vehicle_class = VehicleClass::UUV;
    out.source_path = "builtin:" + out.vehicle_type;
    out.motor.rho = 1028.0;
    out.allocator.rho = 1028.0;

    if (out.archetype == VehicleArchetype::DifferentialDrive)
    {
        out.vehicle_class = VehicleClass::UGV_DIFFERENTIAL;
        out.mass_total = 21.656;
        out.ground_allocator.wheel_radius_m = 0.0686;
        out.ground_allocator.track_width_m = 0.32634;
        out.ground_allocator.max_wheel_angular_speed_radps = 40.0;
        out.ground_allocator.longitudinal_speed_gain_N_per_mps = 180.0;
        validate(out, nullptr);
        return out;
    }

    if (out.archetype == VehicleArchetype::Surface)
    {
        out.vehicle_class = VehicleClass::USV;
        if (out.vehicle_type == "WAMV")
        {
            out.max_thrust_per_thruster_N = 2352.53;
            out.surface_channel_surge_limit_N = 3327.0;
            out.surface_channel_lever_arm_m = 2.85;
            out.mass_total = 242.0;
        }
        else
        {
            out.max_thrust_per_thruster_N = 119.7;
            out.surface_channel_surge_limit_N = 119.7;
            out.mass_total = 80.0;
        }
        validate(out, nullptr);
        return out;
    }
    if (out.archetype == VehicleArchetype::Multirotor)
    {
        out.vehicle_class = VehicleClass::UAV_MULTIROTOR;
        out.mass_total = 2.0;
        // 4 * kT * omega_max^2 from Content/Aero/x500_params.json.
        out.max_total_lift_N = 34.19432;
        validate(out, nullptr);
        return out;
    }
    if (out.archetype == VehicleArchetype::FixedWing)
    {
        out.vehicle_class = VehicleClass::UAV_FIXED_WING;
        out.mass_total = 1.5;
        validate(out, nullptr);
        return out;
    }
    if (out.archetype == VehicleArchetype::VTOL)
    {
        out.vehicle_class = VehicleClass::UAV_VTOL;
        out.mass_total = 5.0;
        // 4 * kT * omega_max^2 from Content/Aero/standard_vtol_params.json.
        out.max_total_lift_N = 180.0;
        validate(out, nullptr);
        return out;
    }
    if (out.archetype == VehicleArchetype::Thruster)
    {
        if (out.vehicle_type == "DesistekSaga")
        {
            out.max_thrust_per_thruster_N = 20.0;
            out.mass_total = 10.0;
        }
        else if (out.vehicle_type == "RexROV2")
        {
            out.max_thrust_per_thruster_N = 1540.0;
            out.mass_total = 1862.87;
        }
        validate(out, nullptr);
        return out;
    }

    if (out.vehicle_type == "EcaA9" || out.vehicle_type == "EcaA9Test")
    {
        out.allocator.S_fin = 0.04155; out.allocator.CL_s = 3.0; out.allocator.CL_r = 3.0;
        out.allocator.x_fin = 0.708; out.allocator.D_prop = 0.17; out.allocator.n_max_rpm = 1000.0;
        out.allocator.max_thrust_N = 200.0; out.allocator.delta_max_deg = 15.0;
        out.motor.D_prop = 0.17; out.motor.rpm_max = 1000.0;
    }
    else if (out.vehicle_type == "LAUV" || out.vehicle_type == "LAUVTest")
    {
        out.allocator.S_fin = 0.0064; out.allocator.CL_s = 3.0; out.allocator.CL_r = 3.0;
        out.allocator.x_fin = 0.40; out.allocator.D_prop = 0.14; out.allocator.n_max_rpm = 9500.0;
        out.allocator.max_thrust_N = 1000.0; out.allocator.delta_max_deg = 15.0;
        out.allocator.u_min = 2.0;
        out.motor.D_prop = 0.14; out.motor.rpm_max = 9500.0;
    }
    else
    {
        return out;
    }

    validate(out, nullptr);
    return out;
}

FossenControlParams load_fossen_control_params(
    const std::string &type,
    const std::string &explicit_path,
    const std::string &params_dir,
    std::string *error)
{
    FossenControlParams params = builtin_fossen_control_params(type);
    if (!params.valid)
    {
        set_error(error, "unsupported vehicle type '" + type + "'");
        return params;
    }

    const std::string filename = fossen_params_filename(type);
    if (filename.empty())
        return params;

    std::string last_error;
    for (const auto &path : candidate_paths(
             filename, explicit_path, params_dir, params.archetype))
    {
        if (path.empty() || !std::filesystem::exists(path))
            continue;
        FossenControlParams candidate = params;
        if (load_json_into(path, candidate, &last_error))
        {
            if (error) error->clear();
            return candidate;
        }
    }

    set_error(error, last_error.empty()
                         ? "Fossen params JSON not found for " + params.vehicle_type
                         : last_error);
    return params;
}

} // namespace hydrox
