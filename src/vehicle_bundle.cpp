// Copyright (c) 2026 OceanX.
#include "vehicle_bundle.h"
#include "bundle_document.h"
#include "control_parameter_fields.h"
#include <algorithm>
#include <cmath>
#include <type_traits>
#if !defined(__NuttX__)
#include <fstream>
#include <sstream>
#endif
namespace hydrox
{
namespace
{
uint64_t fnv1a64(const std::string &bytes)
{
    uint64_t h = 14695981039346656037ULL;
    for (unsigned char c : bytes)
    {
        h ^= c;
        h *= 1099511628211ULL;
    }
    return h;
}
void add_issue(VehicleBundle &b, BundleIssueSeverity severity, const std::string &field,
               const std::string &message)
{
    b.validation.push_back({severity, field, message});
}
struct Contract
{
    const char *archetype;
    const char *vehicle_class;
    VehicleArchetype kind;
    VehicleClass category;
};
constexpr Contract contracts[] = {
    {"slender_body_fin", "uuv", VehicleArchetype::SlenderBodyFin, VehicleClass::UUV},
    {"thruster", "uuv", VehicleArchetype::Thruster, VehicleClass::UUV},
    {"surface", "usv", VehicleArchetype::Surface, VehicleClass::USV},
    {"multirotor", "uav_multirotor", VehicleArchetype::Multirotor, VehicleClass::UAV_MULTIROTOR},
    {"fixed_wing", "uav_fixed_wing", VehicleArchetype::FixedWing, VehicleClass::UAV_FIXED_WING},
    {"vtol", "uav_vtol", VehicleArchetype::VTOL, VehicleClass::UAV_VTOL},
    {"differential_drive", "ugv_differential", VehicleArchetype::DifferentialDrive,
     VehicleClass::UGV_DIFFERENTIAL}};
} // namespace
bool validate_vehicle_bundle(VehicleBundle &bundle, std::string *error)
{
    bundle.validation.clear();
    bundle.logical_actuator_count = bundle.control.archetype == VehicleArchetype::DifferentialDrive
                                        ? 2u
                                        : bundle.control.thrusters.size();
    if (bundle.control.direct_body_wrench)
        bundle.logical_actuator_count = 6;
    else if (bundle.control.archetype == VehicleArchetype::SlenderBodyFin)
        bundle.logical_actuator_count = 5;
    else if (bundle.control.archetype == VehicleArchetype::Surface)
        bundle.logical_actuator_count = 2;
    else if (bundle.control.archetype == VehicleArchetype::Multirotor ||
             bundle.control.archetype == VehicleArchetype::FixedWing)
        bundle.logical_actuator_count = 4;
    else if (bundle.control.archetype == VehicleArchetype::VTOL)
        bundle.logical_actuator_count = 8;
    bundle.allocation_rank = 0;
    if (bundle.schema_version != "2.0")
        add_issue(bundle, BundleIssueSeverity::Error, "schema_version",
                  "only schema version 2.0 is supported");
    if (bundle.id.empty())
        add_issue(bundle, BundleIssueSeverity::Error, "id", "a non-empty bundle id is required");
    if (bundle.control_contract.empty())
        add_issue(bundle, BundleIssueSeverity::Error, "control_contract",
                  "a control-contract identifier is required");

    if (!vehicle_class_matches_archetype(bundle.control.vehicle_class, bundle.control.archetype))
        add_issue(bundle, BundleIssueSeverity::Error, "vehicle_class",
                  "vehicle class is incompatible with the selected control archetype");

    const auto &params = bundle.control;
    visit_control_parameters(params, [&](const char *path, const auto &value) {
        const auto finite = [&](double v) {
            if (!std::isfinite(v))
                add_issue(bundle, BundleIssueSeverity::Error, path, "must be finite");
        };
        if constexpr (std::is_arithmetic_v<std::decay_t<decltype(value)>>)
            finite(value);
        else
            for (double v : value)
                finite(v);
    });
    if (params.mass_total <= 0.0)
        add_issue(bundle, BundleIssueSeverity::Error, "control_model.mass_kg", "must be positive");
    const auto &motor = params.motor;
    if (motor.tau_m <= 0 || motor.rho <= 0 || motor.D_prop <= 0 || motor.rpm_max <= 0 ||
        motor.KT_0 <= 0 || motor.KQ_0 < 0 || motor.eta_motor <= 0 || motor.eta_motor > 1 ||
        motor.V_bat <= 0)
        add_issue(bundle, BundleIssueSeverity::Error, "control_model.motor",
                  "invalid motor model parameters");
    if (params.archetype == VehicleArchetype::SlenderBodyFin)
    {
        if (params.allocator.S_fin <= 0.0 || params.allocator.CL_s <= 0.0 ||
            params.allocator.CL_r <= 0.0 || params.allocator.x_fin <= 0.0 ||
            params.allocator.D_prop <= 0.0 || params.allocator.n_max_rpm <= 0.0 ||
            params.allocator.max_thrust_N <= 0.0 || params.allocator.u_min <= 0.0 ||
            params.allocator.rho <= 0.0 || params.allocator.KT_0 <= 0.0 ||
            params.allocator.delta_max_deg <= 0.0 || params.M44_pitch <= 0.0)
            add_issue(bundle, BundleIssueSeverity::Error, "control_model.actuators",
                      "fin vehicles require positive fin and propeller effectiveness values");
        double mm = 0.0, mn = 0.0, nn = 0.0;
        for (double angle : params.allocator.fin_angles_deg)
        {
            const double a = angle * 3.14159265358979323846 / 180.0;
            const double m = std::cos(a), n = std::sin(a);
            mm += m * m;
            mn += m * n;
            nn += n * n;
        }
        if (!std::isfinite(mm * nn - mn * mn) || mm * nn - mn * mn <= 1e-8)
            add_issue(bundle, BundleIssueSeverity::Error, "control_model.allocator.fin_angles_deg",
                      "fin layout must provide independent pitch and yaw authority");
    }
    else if (params.archetype == VehicleArchetype::Thruster)
    {
        if (params.mass_total <= 0.0)
            add_issue(bundle, BundleIssueSeverity::Error, "control_model.mass_kg",
                      "thruster vehicles require a positive mass");
        if (bundle.control.thrusters.empty() && !params.direct_body_wrench)
            add_issue(bundle, BundleIssueSeverity::Error, "actuator_layout.thrusters",
                      "thruster vehicles require at least one logical thruster");
        if (bundle.control.thrusters.size() > ActuatorCmd{}.ch.size())
            add_issue(bundle, BundleIssueSeverity::Error, "actuator_layout.thrusters",
                      "the current actuator command interface supports at most 8 thrusters");
    }
    else if (params.archetype == VehicleArchetype::Surface)
    {
        if (params.mass_total <= 0.0 || params.max_thrust_per_thruster_N <= 0.0 ||
            params.surface_channel_surge_limit_N <= 0.0 ||
            params.surface_channel_lever_arm_m <= 0.0)
            add_issue(bundle, BundleIssueSeverity::Error, "control_model",
                      "surface vehicles require positive mass and thrust authority");
    }
    else if (params.archetype == VehicleArchetype::DifferentialDrive)
    {
        const auto &ground = params.ground_allocator;
        if (params.mass_total <= 0.0 || ground.wheel_radius_m <= 0.0 ||
            ground.track_width_m <= 0.0 || ground.max_wheel_angular_speed_radps <= 0.0 ||
            ground.longitudinal_speed_gain_N_per_mps <= 0.0)
            add_issue(bundle, BundleIssueSeverity::Error, "control_model",
                      "differential-drive vehicles require positive mass, wheel geometry, speed "
                      "limit, and motor gain");
    }
    else if (params.mass_total <= 0.0)
        add_issue(bundle, BundleIssueSeverity::Error, "control_model.mass_kg",
                  "air vehicles require a positive mass");

    if ((params.archetype == VehicleArchetype::Multirotor ||
         params.archetype == VehicleArchetype::VTOL) &&
        (params.max_total_lift_N <= 0.0 || params.lift_roll_pitch_moment_arm_m <= 0.0 ||
         params.lift_yaw_moment_per_thrust_m <= 0.0))
        add_issue(bundle, BundleIssueSeverity::Error, "control_model.max_total_lift_n",
                  "lift vehicles require positive lift, roll/pitch arm, and yaw authority");
    if (params.archetype == VehicleArchetype::FixedWing)
    {
        const auto &controller = params.fixedwing_gnc;
        if (controller.Ixx_kgm2 <= 0.0 || controller.Iyy_kgm2 <= 0.0 ||
            controller.Izz_kgm2 <= 0.0 || params.fixedwing_allocator.max_forward_thrust_N <= 0.0)
        {
            add_issue(bundle, BundleIssueSeverity::Error, "control_model",
                      "fixed-wing vehicles require positive inertia and pusher authority");
        }
    }
    if (params.archetype == VehicleArchetype::VTOL &&
        params.vtol_allocator.max_pusher_thrust_N <= 0.0)
    {
        add_issue(bundle, BundleIssueSeverity::Error, "control_model.max_forward_thrust_n",
                  "lift-cruise VTOL vehicles require positive pusher authority");
    }

    if (params.archetype == VehicleArchetype::VTOL)
    {
        const auto &allocator = params.vtol_allocator;
        const auto nonnegative_gain = [&](double gain, const char *field) {
            if (gain < 0.0)
                add_issue(bundle, BundleIssueSeverity::Error, field,
                          "must be nonnegative; zero disables the surface");
        };
        nonnegative_gain(allocator.elevator_gain_inv_Nm,
                         "control_model.allocator.elevator_gain_inv_Nm");
        nonnegative_gain(allocator.aileron_gain_inv_Nm,
                         "control_model.allocator.aileron_gain_inv_Nm");
        nonnegative_gain(allocator.rudder_gain_inv_Nm,
                         "control_model.allocator.rudder_gain_inv_Nm");
    }

    for (size_t i = 0; i < bundle.control.thrusters.size(); ++i)
    {
        const auto &thruster = bundle.control.thrusters[i];
        if (!thruster.pos.allFinite() || !thruster.dir.allFinite() || thruster.dir.norm() <= 1e-6)
            add_issue(bundle, BundleIssueSeverity::Error,
                      "actuator_layout.thrusters[" + std::to_string(i) + "].direction_body",
                      "thruster direction must be non-zero");
        if (thruster.max_thrust_N <= 0.0)
            add_issue(bundle, BundleIssueSeverity::Error,
                      "actuator_layout.thrusters[" + std::to_string(i) + "].max_thrust_n",
                      "thruster maximum thrust must be positive");
    }

    if (params.archetype == VehicleArchetype::Thruster && !bundle.control.thrusters.empty() &&
        bundle.validation.empty())
    {
        Eigen::Matrix<double, 6, Eigen::Dynamic> effectiveness(6, bundle.control.thrusters.size());
        for (size_t i = 0; i < bundle.control.thrusters.size(); ++i)
        {
            const auto &thruster = bundle.control.thrusters[i];
            effectiveness.block<3, 1>(0, static_cast<Eigen::Index>(i)) = thruster.dir;
            effectiveness.block<3, 1>(3, static_cast<Eigen::Index>(i)) =
                thruster.pos.cross(thruster.dir);
        }
        bundle.allocation_rank = static_cast<size_t>(effectiveness.fullPivLu().rank());
        if (bundle.allocation_rank < 6)
            add_issue(
                bundle, BundleIssueSeverity::Warning, "actuator_layout.thrusters",
                "layout is under-actuated; only enable controller axes the layout can realise");
    }

    bundle.valid = std::none_of(bundle.validation.begin(), bundle.validation.end(),
                                [](const BundleValidationIssue &issue) {
                                    return issue.severity == BundleIssueSeverity::Error;
                                });
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

VehicleBundle load_vehicle_bundle_json(const std::string &json, const std::string &source_name,
                                       std::string *error)
{
    VehicleBundle b;
    b.source_path = source_name;
    b.fingerprint = fnv1a64(json);
    bundle_json::Document doc(json);
    b.schema_version = doc.string("schema_version");
    b.id = doc.string("id");
    b.control_contract = doc.string("control_contract");
    const std::string archetype = doc.string("archetype"), category = doc.string("vehicle_class");
    const Contract *contract = nullptr;
    for (const auto &c : contracts)
        if (archetype == c.archetype)
            contract = &c;
    if (!contract)
        doc.fail("archetype: unsupported control family");
    else
    {
        b.control.archetype = contract->kind;
        b.control.vehicle_class = contract->category;
        if (category != contract->vehicle_class)
            doc.fail("vehicle_class does not match archetype");
        if (b.control_contract != "hydrox.control/" + archetype + "@2")
            doc.fail("control_contract does not match archetype/version");
    }
    if (b.schema_version != "2.0")
        doc.fail("schema_version: only 2.0 is supported");
    if (b.id.empty() || b.id.size() > 63 ||
        b.id.find_first_not_of(
            "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_") !=
            std::string::npos)
        doc.fail("id: expected 1..63 ASCII identifier characters");
    b.control.vehicle_type = b.id;
    b.control.source_path = source_name;
    if (doc.error.empty())
        visit_control_parameters(b.control,
                                 [&](const char *path, auto &value) { doc.read(path, value); });
    if (doc.error.empty() && b.control.archetype == VehicleArchetype::Thruster)
    {
        const auto kind = doc.string("actuator_layout.kind");
        if (kind == "body_wrench")
            b.control.direct_body_wrench = true;
        else if (kind == "thrusters")
        {
            auto *list = doc.at("actuator_layout.thrusters");
            if (!list || list->kind != bundle_json::Value::Array || list->array.empty() ||
                list->array.size() > 8)
                doc.fail("actuator_layout.thrusters: expected 1..8 thrusters");
            else
                for (auto &entry : list->array)
                {
                    entry.used = true;
                    if (entry.kind != bundle_json::Value::Object || entry.object.size() != 3)
                    {
                        doc.fail("thruster requires position_m, direction_body, max_thrust_n");
                        break;
                    }
                    Thruster thruster;
                    for (const char *key : {"position_m", "direction_body"})
                    {
                        auto it = entry.object.find(key);
                        if (it == entry.object.end() ||
                            it->second.kind != bundle_json::Value::Array ||
                            it->second.array.size() != 3)
                        {
                            doc.fail(std::string("invalid thruster ") + key);
                            break;
                        }
                        it->second.used = true;
                        auto &vec = std::string(key) == "position_m" ? thruster.pos : thruster.dir;
                        for (size_t i = 0; i < 3; ++i)
                        {
                            auto &n = it->second.array[i];
                            n.used = true;
                            if (n.kind != bundle_json::Value::Number)
                                doc.fail("thruster vector must be numeric");
                            vec[i] = n.number;
                        }
                    }
                    auto it = entry.object.find("max_thrust_n");
                    if (it == entry.object.end() || it->second.kind != bundle_json::Value::Number)
                        doc.fail("thruster max_thrust_n must be numeric");
                    else
                    {
                        it->second.used = true;
                        thruster.max_thrust_N = it->second.number;
                    }
                    b.control.thrusters.push_back(thruster);
                }
        }
        else
            doc.fail("actuator_layout.kind: expected thrusters or body_wrench");
    }
    doc.reject_unused(doc.root);
    if (!doc.error.empty())
    {
        if (error)
            *error = doc.error;
        add_issue(b, BundleIssueSeverity::Error, "document", doc.error);
        return b;
    }
    validate_vehicle_bundle(b, error);
    return b;
}
#if !defined(__NuttX__)
VehicleBundle load_vehicle_bundle(const std::string &path, std::string *error)
{
    std::ifstream stream(path, std::ios::binary);
    if (!stream)
    {
        VehicleBundle b;
        b.source_path = path;
        if (error)
            *error = "unable to read vehicle bundle '" + path + "'";
        return b;
    }
    std::ostringstream bytes;
    bytes << stream.rdbuf();
    return load_vehicle_bundle_json(bytes.str(), path, error);
}
#endif
} // namespace hydrox
