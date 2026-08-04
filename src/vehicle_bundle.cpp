#include "vehicle_bundle.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <sstream>

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

    bool read_text_file(const std::string &path, std::string &out)
    {
        std::ifstream file(path, std::ios::binary);
        if (!file)
            return false;
        std::ostringstream buffer;
        buffer << file.rdbuf();
        out = buffer.str();
        return true;
    }

    uint64_t fnv1a64(const std::string &bytes)
    {
        uint64_t hash = 14695981039346656037ULL;
        for (const unsigned char byte : bytes)
        {
            hash ^= static_cast<uint64_t>(byte);
            hash *= 1099511628211ULL;
        }
        return hash;
    }

    std::optional<size_t> key_value_pos(const std::string &json, const std::string &key)
    {
        const std::string needle = "\"" + key + "\"";
        size_t pos = 0;
        while ((pos = json.find(needle, pos)) != std::string::npos)
        {
            size_t cursor = pos + needle.size();
            while (cursor < json.size() && std::isspace(static_cast<unsigned char>(json[cursor])))
                ++cursor;
            if (cursor < json.size() && json[cursor] == ':')
                return cursor + 1;
            pos += needle.size();
        }
        return std::nullopt;
    }

    std::optional<std::string> bracketed_value(const std::string &json, size_t pos,
                                                char open, char close)
    {
        while (pos < json.size() && std::isspace(static_cast<unsigned char>(json[pos])))
            ++pos;
        if (pos >= json.size() || json[pos] != open)
            return std::nullopt;

        const size_t begin = pos;
        int depth = 0;
        bool in_string = false;
        bool escaped = false;
        for (; pos < json.size(); ++pos)
        {
            const char ch = json[pos];
            if (in_string)
            {
                if (escaped)
                    escaped = false;
                else if (ch == '\\')
                    escaped = true;
                else if (ch == '"')
                    in_string = false;
                continue;
            }
            if (ch == '"')
                in_string = true;
            else if (ch == open)
                ++depth;
            else if (ch == close && --depth == 0)
                return json.substr(begin, pos - begin + 1);
        }
        return std::nullopt;
    }

    std::optional<std::string> object_for_key(const std::string &json, const std::string &key)
    {
        const auto pos = key_value_pos(json, key);
        return pos ? bracketed_value(json, *pos, '{', '}') : std::nullopt;
    }

    std::optional<std::string> array_for_key(const std::string &json, const std::string &key)
    {
        const auto pos = key_value_pos(json, key);
        return pos ? bracketed_value(json, *pos, '[', ']') : std::nullopt;
    }

    std::optional<std::string> string_for_key(const std::string &json, const std::string &key)
    {
        const auto value_pos = key_value_pos(json, key);
        if (!value_pos)
            return std::nullopt;
        size_t pos = *value_pos;
        while (pos < json.size() && std::isspace(static_cast<unsigned char>(json[pos])))
            ++pos;
        if (pos >= json.size() || json[pos] != '"')
            return std::nullopt;
        const size_t begin = ++pos;
        bool escaped = false;
        for (; pos < json.size(); ++pos)
        {
            if (!escaped && json[pos] == '"')
                return json.substr(begin, pos - begin);
            escaped = !escaped && json[pos] == '\\';
            if (json[pos] != '\\')
                escaped = false;
        }
        return std::nullopt;
    }

    std::optional<double> number_for_key(const std::string &json, const std::string &key)
    {
        const auto value_pos = key_value_pos(json, key);
        if (!value_pos)
            return std::nullopt;
        size_t pos = *value_pos;
        while (pos < json.size() && std::isspace(static_cast<unsigned char>(json[pos])))
            ++pos;
        char *end = nullptr;
        const double value = std::strtod(json.c_str() + pos, &end);
        return end == json.c_str() + pos ? std::nullopt : std::optional<double>(value);
    }

    std::vector<double> numbers_in_array(const std::string &text)
    {
        std::vector<double> values;
        const char *cursor = text.c_str();
        const char *const end = cursor + text.size();
        while (cursor < end)
        {
            while (cursor < end && !(std::isdigit(static_cast<unsigned char>(*cursor)) ||
                                     *cursor == '-' || *cursor == '+' || *cursor == '.'))
                ++cursor;
            if (cursor >= end)
                break;
            char *next = nullptr;
            const double value = std::strtod(cursor, &next);
            if (next == cursor)
            {
                ++cursor;
                continue;
            }
            values.push_back(value);
            cursor = next;
        }
        return values;
    }

    std::vector<std::string> objects_in_array(const std::string &array)
    {
        std::vector<std::string> objects;
        size_t pos = 0;
        while ((pos = array.find('{', pos)) != std::string::npos)
        {
            const auto object = bracketed_value(array, pos, '{', '}');
            if (!object)
                break;
            objects.push_back(*object);
            pos += object->size();
        }
        return objects;
    }

    std::optional<std::array<double, 3>> vector3_for_key(const std::string &json,
                                                           const std::string &key)
    {
        const auto array = array_for_key(json, key);
        if (!array)
            return std::nullopt;
        const auto values = numbers_in_array(*array);
        if (values.size() != 3)
            return std::nullopt;
        return std::array<double, 3>{values[0], values[1], values[2]};
    }

    void add_issue(VehicleBundle &bundle, BundleIssueSeverity severity,
                   const std::string &field, const std::string &message)
    {
        bundle.validation.push_back({severity, field, message});
    }

    void assign_if_present(const std::string &json, const std::string &key, double &out)
    {
        if (const auto value = number_for_key(json, key))
            out = *value;
    }

    void apply_fin_controller(const std::string &json, FossenControlParams &params)
    {
        if (const auto depth = object_for_key(json, "depth"))
        {
            assign_if_present(*depth, "kp", params.gnc.depth.kp);
            assign_if_present(*depth, "kd", params.gnc.depth.kd);
            assign_if_present(*depth, "ki", params.gnc.depth.ki);
            assign_if_present(*depth, "wn", params.gnc.depth.wn);
            assign_if_present(*depth, "zeta", params.gnc.depth.zeta);
            assign_if_present(*depth, "vmax", params.gnc.depth.vmax);
            assign_if_present(*depth, "ei_max", params.gnc.depth.ei_max);
            assign_if_present(*depth, "theta_max", params.gnc.depth.theta_max);
        }
        if (const auto pitch = object_for_key(json, "pitch"))
        {
            assign_if_present(*pitch, "kp", params.gnc.pitch.kp);
            assign_if_present(*pitch, "kd", params.gnc.pitch.kd);
            assign_if_present(*pitch, "ki", params.gnc.pitch.ki);
            assign_if_present(*pitch, "ei_max", params.gnc.pitch.ei_max);
            assign_if_present(*pitch, "tau_max", params.gnc.pitch.tau_max);
        }
        if (const auto surge = object_for_key(json, "surge"))
        {
            assign_if_present(*surge, "kp", params.gnc.surge.kp);
            assign_if_present(*surge, "ki", params.gnc.surge.ki);
            assign_if_present(*surge, "kd", params.gnc.surge.kd);
            assign_if_present(*surge, "drag_ff", params.gnc.surge.drag_ff);
            assign_if_present(*surge, "forward_min_tau", params.gnc.surge.forward_min_tau);
            assign_if_present(*surge, "tau_rate_limit", params.gnc.surge.tau_rate_limit);
        }
    }

    void apply_thruster_controller(const std::string &json, FossenControlParams &params)
    {
        const auto controller = object_for_key(json, "thruster");
        if (!controller)
            return;
        const auto apply_axis = [](const std::optional<std::string> &axis,
                                   ThrusterVehicleController::Axis &out)
        {
            if (!axis)
                return;
            assign_if_present(*axis, "kp", out.kp);
            assign_if_present(*axis, "kd", out.kd);
            assign_if_present(*axis, "accel_max", out.accel_max);
        };
        apply_axis(object_for_key(*controller, "surge"), params.thruster_gnc.surge);
        apply_axis(object_for_key(*controller, "sway"), params.thruster_gnc.sway);
        apply_axis(object_for_key(*controller, "heave"), params.thruster_gnc.heave);
        apply_axis(object_for_key(*controller, "roll"), params.thruster_gnc.roll);
        apply_axis(object_for_key(*controller, "pitch"), params.thruster_gnc.pitch);
        apply_axis(object_for_key(*controller, "yaw"), params.thruster_gnc.yaw);
        params.archetype_control_loaded_from_json = true;
    }

    void apply_surface_controller(const std::string &json, FossenControlParams &params)
    {
        const auto controller = object_for_key(json, "surface");
        if (!controller)
            return;
        assign_if_present(*controller, "surge_kp", params.surface_gnc.surge_kp);
        assign_if_present(*controller, "surge_kd", params.surface_gnc.surge_kd);
        assign_if_present(*controller, "surge_ki", params.surface_gnc.surge_ki);
        assign_if_present(*controller, "surge_integral_limit", params.surface_gnc.surge_integral_limit);
        assign_if_present(*controller, "yaw_kp", params.surface_gnc.yaw_kp);
        assign_if_present(*controller, "yaw_kd", params.surface_gnc.yaw_kd);
        assign_if_present(*controller, "max_force_n", params.surface_gnc.max_force_N);
        assign_if_present(*controller, "max_moment_nm", params.surface_gnc.max_moment_Nm);
        assign_if_present(*controller, "waypoint_surge_mps", params.surface_gnc.waypoint_surge_mps);
        assign_if_present(*controller, "sideslip_compensation_gain", params.surface_gnc.sideslip_compensation_gain);
        assign_if_present(*controller, "max_crab_angle_rad", params.surface_gnc.max_crab_angle_rad);
        params.archetype_control_loaded_from_json = true;
    }

    void apply_ground_controller(const std::string &json, FossenControlParams &params)
    {
        const auto controller = object_for_key(json, "differential_drive");
        if (!controller)
            return;
        auto &out = params.ground_gnc;
        assign_if_present(*controller, "surge_kp", out.surge_kp);
        assign_if_present(*controller, "surge_ki", out.surge_ki);
        assign_if_present(*controller, "surge_integral_limit", out.surge_integral_limit);
        assign_if_present(*controller, "yaw_heading_kp", out.yaw_heading_kp);
        assign_if_present(*controller, "yaw_rate_kp", out.yaw_rate_kp);
        assign_if_present(*controller, "max_force_n", out.max_force_N);
        assign_if_present(*controller, "max_moment_nm", out.max_moment_Nm);
        assign_if_present(*controller, "waypoint_surge_mps", out.waypoint_surge_mps);
        assign_if_present(*controller, "waypoint_stop_radius_m", out.waypoint_stop_radius_m);
        assign_if_present(*controller, "waypoint_slowdown_m", out.waypoint_slowdown_m);
        assign_if_present(*controller, "max_yaw_rate_radps", out.max_yaw_rate_radps);
        params.archetype_control_loaded_from_json = true;
    }

    void apply_multirotor_controller(const std::string &json, FossenControlParams &params)
    {
        const auto controller = object_for_key(json, "multirotor");
        if (!controller)
            return;
        auto &out = params.multirotor_gnc;
        assign_if_present(*controller, "z_kp", out.z_kp);
        assign_if_present(*controller, "z_kd", out.z_kd);
        assign_if_present(*controller, "roll_kp", out.roll_kp);
        assign_if_present(*controller, "roll_kd", out.roll_kd);
        assign_if_present(*controller, "pitch_kp", out.pitch_kp);
        assign_if_present(*controller, "pitch_kd", out.pitch_kd);
        assign_if_present(*controller, "yaw_kp", out.yaw_kp);
        assign_if_present(*controller, "yaw_kd", out.yaw_kd);
        assign_if_present(*controller, "xy_kp", out.xy_kp);
        assign_if_present(*controller, "xy_kd", out.xy_kd);
        assign_if_present(*controller, "max_tilt_rad", out.max_tilt_rad);
        assign_if_present(*controller, "max_xy_accel", out.max_xy_accel);
        assign_if_present(*controller, "max_z_accel", out.max_z_accel);
        params.archetype_control_loaded_from_json = true;
    }

    void apply_fixedwing_controller(const std::string &json, FossenControlParams &params)
    {
        auto controller = object_for_key(json, "fixed_wing");
        // Runtime profiles historically used "fixedwing", while reusable
        // bundles use "fixed_wing".  Accept both so vehicle-specific tuning
        // never silently falls back to generic controller defaults.
        if (!controller)
            controller = object_for_key(json, "fixedwing");
        if (!controller)
            return;
        auto &out = params.fixedwing_gnc;
        assign_if_present(*controller, "cruise_speed_mps", out.cruise_speed_mps);
        assign_if_present(*controller, "altitude_kp", out.altitude_kp);
        assign_if_present(*controller, "altitude_kd", out.altitude_kd);
        assign_if_present(*controller, "altitude_ki", out.altitude_ki);
        assign_if_present(*controller, "altitude_integral_limit", out.altitude_integral_limit);
        assign_if_present(*controller, "pitch_limit_rad", out.pitch_limit_rad);
        assign_if_present(*controller, "course_kp", out.course_kp);
        assign_if_present(*controller, "roll_limit_rad", out.roll_limit_rad);
        assign_if_present(*controller, "roll_attitude_kp", out.roll_attitude_kp);
        assign_if_present(*controller, "roll_rate_kd", out.roll_rate_kd);
        assign_if_present(*controller, "max_roll_command", out.max_roll_command);
        assign_if_present(*controller, "pitch_attitude_kp", out.pitch_attitude_kp);
        assign_if_present(*controller, "pitch_rate_kd", out.pitch_rate_kd);
        assign_if_present(*controller, "pitch_trim", out.pitch_trim);
        assign_if_present(*controller, "max_pitch_command", out.max_pitch_command);
        auto &allocator = params.fixedwing_allocator;
        assign_if_present(*controller, "elevator_kp", allocator.elevator_kp);
        assign_if_present(*controller, "elevator_kd", allocator.elevator_kd);
        assign_if_present(*controller, "aileron_kp", allocator.aileron_kp);
        assign_if_present(*controller, "aileron_kd", allocator.aileron_kd);
        assign_if_present(*controller, "rudder_kp", allocator.rudder_kp);
        assign_if_present(*controller, "throttle_trim", allocator.throttle_trim);
        assign_if_present(*controller, "throttle_kp", allocator.throttle_kp);
        params.archetype_control_loaded_from_json = true;
    }

    void apply_vtol_controller(const std::string &json, FossenControlParams &params)
    {
        const auto controller = object_for_key(json, "vtol");
        if (!controller)
            return;
        auto &out = params.vtol_gnc;
        assign_if_present(*controller, "z_kp", out.z_kp);
        assign_if_present(*controller, "z_kd", out.z_kd);
        assign_if_present(*controller, "roll_kp", out.roll_kp);
        assign_if_present(*controller, "roll_kd", out.roll_kd);
        assign_if_present(*controller, "pitch_kp", out.pitch_kp);
        assign_if_present(*controller, "pitch_kd", out.pitch_kd);
        assign_if_present(*controller, "yaw_kp", out.yaw_kp);
        assign_if_present(*controller, "yaw_kd", out.yaw_kd);
        assign_if_present(*controller, "xy_kp", out.xy_kp);
        assign_if_present(*controller, "xy_kd", out.xy_kd);
        assign_if_present(*controller, "max_tilt_rad", out.max_tilt_rad);
        assign_if_present(*controller, "cruise_speed_mps", out.cruise_speed_mps);
        params.archetype_control_loaded_from_json = true;
    }

    bool class_matches_archetype(VehicleClass vehicle_class, VehicleArchetype archetype)
    {
        switch (archetype)
        {
        case VehicleArchetype::SlenderBodyFin:
        case VehicleArchetype::Thruster:
            return vehicle_class == VehicleClass::UUV;
        case VehicleArchetype::Surface:
            return vehicle_class == VehicleClass::USV;
        case VehicleArchetype::Multirotor:
            return vehicle_class == VehicleClass::UAV_MULTIROTOR;
        case VehicleArchetype::FixedWing:
            return vehicle_class == VehicleClass::UAV_FIXED_WING;
        case VehicleArchetype::VTOL:
            return vehicle_class == VehicleClass::UAV_VTOL;
        case VehicleArchetype::DifferentialDrive:
            return vehicle_class == VehicleClass::UGV_DIFFERENTIAL;
        }
        return false;
    }

    VehicleArchetype archetype_from_contract(const std::string &value, bool &ok)
    {
        const std::string key = compact_key(value);
        ok = true;
        if (key == "slenderbodyfin" || key == "auvfin") return VehicleArchetype::SlenderBodyFin;
        if (key == "thruster" || key == "thruster6dof" || key == "rov") return VehicleArchetype::Thruster;
        if (key == "surface" || key == "surfacevessel" || key == "usv") return VehicleArchetype::Surface;
        if (key == "multirotor" || key == "quadrotor") return VehicleArchetype::Multirotor;
        if (key == "fixedwing") return VehicleArchetype::FixedWing;
        if (key == "vtol" || key == "liftcruise") return VehicleArchetype::VTOL;
        if (key == "differentialdrive" || key == "skidsteer" || key == "ugv")
            return VehicleArchetype::DifferentialDrive;
        ok = false;
        return VehicleArchetype::SlenderBodyFin;
    }

    VehicleClass vehicle_class_from_contract(const std::string &value, bool &ok)
    {
        const std::string key = compact_key(value);
        ok = true;
        if (key == "uuv" || key == "marine") return VehicleClass::UUV;
        if (key == "usv" || key == "surface") return VehicleClass::USV;
        if (key == "uavmultirotor" || key == "multirotor") return VehicleClass::UAV_MULTIROTOR;
        if (key == "uavfixedwing" || key == "fixedwing") return VehicleClass::UAV_FIXED_WING;
        if (key == "uavvtol" || key == "vtol") return VehicleClass::UAV_VTOL;
        if (key == "ugv" || key == "ugvdifferential" || key == "ground")
            return VehicleClass::UGV_DIFFERENTIAL;
        ok = false;
        return VehicleClass::UUV;
    }

    FossenControlParams defaults_for(VehicleArchetype archetype)
    {
        switch (archetype)
        {
        case VehicleArchetype::Thruster: return builtin_fossen_control_params("DesistekSaga");
        case VehicleArchetype::Surface: return builtin_fossen_control_params("SurfaceVessel");
        case VehicleArchetype::Multirotor: return builtin_fossen_control_params("X500");
        case VehicleArchetype::FixedWing: return builtin_fossen_control_params("RCCessna");
        case VehicleArchetype::VTOL: return builtin_fossen_control_params("StandardVTOL");
        case VehicleArchetype::DifferentialDrive: return builtin_fossen_control_params("R1Rover");
        case VehicleArchetype::SlenderBodyFin:
        default: return builtin_fossen_control_params("EcaA9");
        }
    }
} // namespace

bool validate_vehicle_bundle(VehicleBundle &bundle, std::string *error)
{
    bundle.validation.clear();
    bundle.logical_actuator_count =
        bundle.control.archetype == VehicleArchetype::DifferentialDrive
            ? 2u : bundle.thrusters.size();
    bundle.allocation_rank = 0;
    if (bundle.schema_version != "1.0")
        add_issue(bundle, BundleIssueSeverity::Error, "schema_version",
                  "only schema version 1.0 is supported");
    if (bundle.id.empty())
        add_issue(bundle, BundleIssueSeverity::Error, "id", "a non-empty bundle id is required");
    if (bundle.control_contract.empty())
        add_issue(bundle, BundleIssueSeverity::Error, "control_contract",
                  "a control-contract identifier is required");

    if (!class_matches_archetype(bundle.control.vehicle_class, bundle.control.archetype))
        add_issue(bundle, BundleIssueSeverity::Error, "vehicle_class",
                  "vehicle class is incompatible with the selected control archetype");

    const auto &params = bundle.control;
    if (params.archetype == VehicleArchetype::SlenderBodyFin)
    {
        if (params.allocator.S_fin <= 0.0 || params.allocator.CL_s <= 0.0 ||
            params.allocator.CL_r <= 0.0 || params.allocator.x_fin <= 0.0 ||
            params.allocator.D_prop <= 0.0 || params.allocator.n_max_rpm <= 0.0 ||
            params.allocator.max_thrust_N <= 0.0)
            add_issue(bundle, BundleIssueSeverity::Error, "control_model.actuators",
                      "fin vehicles require positive fin and propeller effectiveness values");
    }
    else if (params.archetype == VehicleArchetype::Thruster)
    {
        if (params.mass_total <= 0.0)
            add_issue(bundle, BundleIssueSeverity::Error, "control_model.mass_kg",
                      "thruster vehicles require a positive mass");
        if (bundle.thrusters.empty())
            add_issue(bundle, BundleIssueSeverity::Error, "actuator_layout.thrusters",
                      "thruster vehicles require at least one logical thruster");
        if (bundle.thrusters.size() > ActuatorCmd{}.ch.size())
            add_issue(bundle, BundleIssueSeverity::Error, "actuator_layout.thrusters",
                      "the current actuator command interface supports at most 8 thrusters");
    }
    else if (params.archetype == VehicleArchetype::Surface)
    {
        if (params.mass_total <= 0.0 || params.max_thrust_per_thruster_N <= 0.0)
            add_issue(bundle, BundleIssueSeverity::Error, "control_model",
                      "surface vehicles require positive mass and thrust authority");
    }
    else if (params.archetype == VehicleArchetype::DifferentialDrive)
    {
        const auto &ground = params.ground_allocator;
        if (params.mass_total <= 0.0 || ground.wheel_radius_m <= 0.0 ||
            ground.track_width_m <= 0.0 ||
            ground.max_wheel_angular_speed_radps <= 0.0 ||
            ground.longitudinal_speed_gain_N_per_mps <= 0.0)
            add_issue(bundle, BundleIssueSeverity::Error, "control_model",
                      "differential-drive vehicles require positive mass, wheel geometry, speed limit, and motor gain");
    }
    else if (params.mass_total <= 0.0)
        add_issue(bundle, BundleIssueSeverity::Error, "control_model.mass_kg",
                  "air vehicles require a positive mass");

    if ((params.archetype == VehicleArchetype::Multirotor ||
         params.archetype == VehicleArchetype::VTOL) && params.max_total_lift_N <= 0.0)
        add_issue(bundle, BundleIssueSeverity::Error, "control_model.max_total_lift_n",
                  "lift vehicles require positive total lift authority");

    for (size_t i = 0; i < bundle.thrusters.size(); ++i)
    {
        const auto &thruster = bundle.thrusters[i];
        if (thruster.dir.norm() <= 1e-6)
            add_issue(bundle, BundleIssueSeverity::Error,
                      "actuator_layout.thrusters[" + std::to_string(i) + "].direction_body",
                      "thruster direction must be non-zero");
        if (thruster.max_thrust_N <= 0.0)
            add_issue(bundle, BundleIssueSeverity::Error,
                      "actuator_layout.thrusters[" + std::to_string(i) + "].max_thrust_n",
                      "thruster maximum thrust must be positive");
    }

    if (params.archetype == VehicleArchetype::Thruster && !bundle.thrusters.empty())
    {
        Eigen::Matrix<double, 6, Eigen::Dynamic> effectiveness(6, bundle.thrusters.size());
        for (size_t i = 0; i < bundle.thrusters.size(); ++i)
        {
            const auto &thruster = bundle.thrusters[i];
            effectiveness.block<3, 1>(0, static_cast<Eigen::Index>(i)) = thruster.dir;
            effectiveness.block<3, 1>(3, static_cast<Eigen::Index>(i)) =
                thruster.pos.cross(thruster.dir);
        }
        bundle.allocation_rank = static_cast<size_t>(effectiveness.fullPivLu().rank());
        if (bundle.allocation_rank < 6)
            add_issue(bundle, BundleIssueSeverity::Warning, "actuator_layout.thrusters",
                      "layout is under-actuated; only enable controller axes the layout can realise");
    }

    bundle.valid = std::none_of(bundle.validation.begin(), bundle.validation.end(),
        [](const BundleValidationIssue &issue) { return issue.severity == BundleIssueSeverity::Error; });
    bundle.control.valid = bundle.valid;
    if (!bundle.valid && error)
    {
        for (const auto &issue : bundle.validation)
            if (issue.severity == BundleIssueSeverity::Error)
            {
                *error = issue.field + ": " + issue.message;
                break;
            }
    }
    else if (error)
        error->clear();
    return bundle.valid;
}

VehicleBundle load_vehicle_bundle(const std::string &path, std::string *error)
{
    VehicleBundle bundle;
    bundle.source_path = path;
    std::string json;
    if (!read_text_file(path, json))
    {
        if (error) *error = "unable to read vehicle bundle '" + path + "'";
        return bundle;
    }
    bundle.fingerprint = fnv1a64(json);

    bundle.schema_version = string_for_key(json, "schema_version").value_or("");
    bundle.id = string_for_key(json, "id").value_or("");
    bundle.control_contract = string_for_key(json, "control_contract").value_or("");
    bool archetype_ok = false;
    bundle.control.archetype = archetype_from_contract(
        string_for_key(json, "archetype").value_or(""), archetype_ok);
    if (!archetype_ok)
    {
        add_issue(bundle, BundleIssueSeverity::Error, "archetype",
                  "unsupported or missing control archetype");
        if (error) *error = "archetype: unsupported or missing control archetype";
        return bundle;
    }

    bundle.control = defaults_for(bundle.control.archetype);
    bundle.control.archetype = archetype_from_contract(
        string_for_key(json, "archetype").value_or(""), archetype_ok);
    bool class_ok = false;
    bundle.control.vehicle_class = vehicle_class_from_contract(
        string_for_key(json, "vehicle_class").value_or(""), class_ok);
    if (!class_ok)
    {
        if (error) *error = "vehicle_class: unsupported or missing vehicle class";
        return bundle;
    }
    bundle.control.vehicle_type = bundle.id;
    bundle.control.source_path = path;
    bundle.control.loaded_from_json = true;

    const auto model = object_for_key(json, "control_model");
    if (!model)
    {
        add_issue(bundle, BundleIssueSeverity::Error, "control_model", "object is required");
        if (error) *error = "control_model: object is required";
        return bundle;
    }
    assign_if_present(*model, "mass_kg", bundle.control.mass_total);
    assign_if_present(*model, "pitch_inertia_kg_m2", bundle.control.M44_pitch);
    assign_if_present(*model, "max_thrust_per_actuator_n", bundle.control.max_thrust_per_thruster_N);
    assign_if_present(*model, "max_total_lift_n", bundle.control.max_total_lift_N);

    if (bundle.control.archetype == VehicleArchetype::SlenderBodyFin)
    {
        if (const auto fins = object_for_key(*model, "fins"))
        {
            assign_if_present(*fins, "area_m2", bundle.control.allocator.S_fin);
            assign_if_present(*fins, "lift_slope_pitch_per_rad", bundle.control.allocator.CL_s);
            assign_if_present(*fins, "lift_slope_yaw_per_rad", bundle.control.allocator.CL_r);
            assign_if_present(*fins, "moment_arm_m", bundle.control.allocator.x_fin);
            assign_if_present(*fins, "max_deflection_deg", bundle.control.allocator.delta_max_deg);
            assign_if_present(*fins, "minimum_effective_speed_mps", bundle.control.allocator.u_min);
        }
        if (const auto propeller = object_for_key(*model, "propeller"))
        {
            assign_if_present(*propeller, "diameter_m", bundle.control.allocator.D_prop);
            assign_if_present(*propeller, "max_rpm", bundle.control.allocator.n_max_rpm);
            assign_if_present(*propeller, "max_thrust_n", bundle.control.allocator.max_thrust_N);
            assign_if_present(*propeller, "thrust_coefficient", bundle.control.allocator.KT_0);
            assign_if_present(*propeller, "torque_coefficient", bundle.control.motor.KQ_0);
            bundle.control.motor.D_prop = bundle.control.allocator.D_prop;
            bundle.control.motor.rpm_max = bundle.control.allocator.n_max_rpm;
            bundle.control.motor.KT_0 = bundle.control.allocator.KT_0;
        }
        if (const auto controller = object_for_key(*model, "controller"))
            apply_fin_controller(*controller, bundle.control);
    }
    else if (bundle.control.archetype == VehicleArchetype::Thruster)
    {
        if (const auto controller = object_for_key(*model, "controller"))
            apply_thruster_controller(*controller, bundle.control);
        if (const auto layout = object_for_key(json, "actuator_layout"))
        {
            if (const auto thrusters = array_for_key(*layout, "thrusters"))
            {
                for (const auto &entry : objects_in_array(*thrusters))
                {
                    const auto pos = vector3_for_key(entry, "position_m");
                    const auto dir = vector3_for_key(entry, "direction_body");
                    const auto max = number_for_key(entry, "max_thrust_n");
                    if (!pos || !dir || !max)
                    {
                        if (error)
                            *error = "actuator_layout.thrusters: each thruster requires "
                                     "position_m, direction_body, and max_thrust_n";
                        return bundle;
                    }
                    Thruster thruster;
                    thruster.pos = Eigen::Vector3d((*pos)[0], (*pos)[1], (*pos)[2]);
                    thruster.dir = Eigen::Vector3d((*dir)[0], (*dir)[1], (*dir)[2]);
                    thruster.max_thrust_N = *max;
                    if (thruster.dir.norm() > 1e-6)
                        thruster.dir.normalize();
                    bundle.thrusters.push_back(thruster);
                }
            }
        }
    }
    else if (bundle.control.archetype == VehicleArchetype::Surface)
    {
        assign_if_present(*model, "channel_surge_limit_n",
                          bundle.control.surface_channel_surge_limit_N);
        assign_if_present(*model, "channel_lever_arm_m",
                          bundle.control.surface_channel_lever_arm_m);
        if (const auto controller = object_for_key(*model, "controller"))
            apply_surface_controller(*controller, bundle.control);
    }
    else if (bundle.control.archetype == VehicleArchetype::DifferentialDrive)
    {
        auto &ground = bundle.control.ground_allocator;
        assign_if_present(*model, "wheel_radius_m", ground.wheel_radius_m);
        assign_if_present(*model, "track_width_m", ground.track_width_m);
        assign_if_present(*model, "max_wheel_angular_speed_radps",
                          ground.max_wheel_angular_speed_radps);
        assign_if_present(*model, "longitudinal_speed_gain_n_per_mps",
                          ground.longitudinal_speed_gain_N_per_mps);
        if (const auto controller = object_for_key(*model, "controller"))
            apply_ground_controller(*controller, bundle.control);
    }
    else if (bundle.control.archetype == VehicleArchetype::Multirotor)
    {
        if (const auto controller = object_for_key(*model, "controller"))
            apply_multirotor_controller(*controller, bundle.control);
    }
    else if (bundle.control.archetype == VehicleArchetype::FixedWing)
    {
        if (const auto controller = object_for_key(*model, "controller"))
            apply_fixedwing_controller(*controller, bundle.control);
    }
    else if (bundle.control.archetype == VehicleArchetype::VTOL)
    {
        if (const auto controller = object_for_key(*model, "controller"))
            apply_vtol_controller(*controller, bundle.control);
    }

    validate_vehicle_bundle(bundle, error);
    return bundle;
}
} // namespace hydrox
