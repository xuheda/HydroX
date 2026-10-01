#include "sitl_xlog.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace hydrox::sitl
{
namespace
{
    constexpr uint64_t kSetpointHeartbeatPeriodNs = 1'000'000'000ULL;

    double wrap_pi(double angle)
    {
        constexpr double kPi = 3.14159265358979323846;
        while (angle > kPi)
            angle -= 2.0 * kPi;
        while (angle < -kPi)
            angle += 2.0 * kPi;
        return angle;
    }

    std::string timestamp_tag()
    {
        const std::time_t time = std::time(nullptr);
        std::tm local{};
#ifdef _WIN32
        localtime_s(&local, &time);
#else
        localtime_r(&time, &local);
#endif
        std::ostringstream stream;
        stream << std::put_time(&local, "%Y%m%d_%H%M%S");
        return stream.str();
    }

    std::filesystem::path default_path(const Config &config)
    {
        return std::filesystem::path(config.log_directory.empty() ? "log" : config.log_directory) /
               ("xlog_" + timestamp_tag() + "_" + std::to_string(xlog::unix_time_ns_now()) + ".xlog");
    }

    bool xlog_auto(const std::string &value)
    {
        return value == "auto" || value == "AUTO";
    }

    std::string safe_file_component(std::string value)
    {
        for (char &ch : value)
        {
            const unsigned char byte = static_cast<unsigned char>(ch);
            if (!std::isalnum(byte) && ch != '-' && ch != '_')
                ch = '_';
        }
        return value.empty() ? "vehicle" : value;
    }

    int active_actuator_count(const ControlParameters &params)
    {
        switch (params.archetype)
        {
        case VehicleArchetype::Thruster:
            return params.direct_body_wrench ? 6 : static_cast<int>(params.thrusters.size());
        case VehicleArchetype::DifferentialDrive:
        case VehicleArchetype::Surface:
            return 2;
        case VehicleArchetype::Multirotor:
        case VehicleArchetype::FixedWing:
            return 4;
        case VehicleArchetype::VTOL:
            return 8;
        case VehicleArchetype::SlenderBodyFin:
        default:
            return 5;
        }
    }

    std::string metadata_json(const Config &config,
                              const ControlParameters &params,
                              AccelMode accel_mode,
                              const XLogIdentity &identity)
    {
        const auto bytes_hex = [](const auto &bytes)
        {
            std::ostringstream hex;
            hex << std::hex << std::setfill('0');
            for (const uint8_t byte : bytes)
                hex << std::setw(2) << static_cast<unsigned int>(byte);
            return hex.str();
        };
        const runtime::HilSessionConfigV1 &session = identity.session_config;
        const runtime::HilSessionConfigPayload session_payload =
            runtime::encode_hil_session_config(session);
        std::ostringstream stream;
        stream << "{"
               << "\"format\":\"XLog\","
               << "\"format_version\":\"1.0\","
               << "\"producer\":\"hydrox_sitl\","
               << "\"run_id\":\"" << xlog::json_escape(config.run_id) << "\","
               << "\"recording_policy\":\"process_lifetime\","
               << "\"vehicle\":\"" << xlog::json_escape(config.vehicle) << "\","
               << "\"vehicle_type\":\"" << xlog::json_escape(config.vehicle_type) << "\","
               << "\"resolved_vehicle_type\":\"" << xlog::json_escape(params.vehicle_type) << "\","
               << "\"vehicle_class\":\"" << vehicle_class_name(params.vehicle_class) << "\","
               << "\"vehicle_archetype\":\"" << vehicle_archetype_name(params.archetype) << "\","
               << "\"params_source\":\"" << xlog::json_escape(params.source_path) << "\","
               << "\"profile_id\":\"" << xlog::json_escape(identity.profile_id) << "\","
               << "\"control_contract\":\"" << xlog::json_escape(identity.control_contract) << "\","
               << "\"profile_bound\":" << (identity.profile_bound ? "true" : "false") << ','
               << "\"profile_fingerprint\":\"" << std::hex << std::setw(16)
               << std::setfill('0') << identity.profile_fingerprint << std::dec << "\","
               << "\"recording_session_id\":\""
               << xlog::json_escape(identity.recording_session_id) << "\","
               << "\"session_digest\":\"" << bytes_hex(identity.session_digest) << "\","
               << "\"session_config_payload\":\"" << bytes_hex(session_payload) << "\","
               << "\"session_config\":{"
               << "\"schema_version\":" << session.schema_version << ','
               << "\"flags\":" << session.flags << ','
               << "\"nominal_period_us\":" << session.nominal_period_us << ','
               << "\"max_sensor_dt_us\":" << session.max_sensor_dt_us << ','
               << "\"sensor_timeout_us\":" << session.sensor_timeout_us << ','
               << "\"setpoint_timeout_us\":" << session.setpoint_timeout_us << ','
               << "\"initial_n_mm\":" << session.initial_n_mm << ','
               << "\"initial_e_mm\":" << session.initial_e_mm << ','
               << "\"initial_down_mm\":" << session.initial_down_mm << ','
               << "\"initial_heading_urad\":" << session.initial_heading_urad << ','
               << "\"initial_surge_mmps\":" << session.initial_surge_mmps << ','
               << "\"gps_origin_lat_e7\":" << session.gps_origin_lat_e7 << ','
               << "\"gps_origin_lon_e7\":" << session.gps_origin_lon_e7 << ','
               << "\"gps_origin_alt_mm\":" << session.gps_origin_alt_mm << ','
               << "\"gps_max_radius_mm\":" << session.gps_max_radius_mm << ','
               << "\"mission_radius_mm\":" << session.mission_radius_mm << ','
               << "\"required_sensor_mask\":" << session.required_sensor_mask << ','
               << "\"accel_mode\":" << static_cast<unsigned int>(session.accel_mode) << ','
               << "\"feedback_source\":" << static_cast<unsigned int>(session.feedback_source)
               << "},"
               << "\"ue5_host\":\"" << xlog::json_escape(config.ue5_host) << "\","
               << "\"ue5_port\":" << config.ue5_port << ','
               << "\"dds_host\":\"" << xlog::json_escape(config.dds_host) << "\","
               << "\"dds_port\":" << config.dds_port << ','
               << "\"ros_domain_id\":" << config.ros_domain_id << ','
               << "\"rate_hz\":" << config.rate_hz << ','
               << "\"residual_policy\":{"
               << "\"mode\":\"" << residual_policy_mode_name(config.residual_policy_mode) << "\","
               << "\"host\":\"" << xlog::json_escape(config.residual_policy_host) << "\","
               << "\"remote_port\":" << config.residual_policy_port << ','
               << "\"local_port\":" << config.residual_policy_local_port << ','
               << "\"nonce\":" << config.residual_policy_nonce << ','
               << "\"policy_hz\":" << config.residual_policy_hz << ','
               << "\"timeout_ms\":" << config.residual_policy_timeout_ms << ','
               << "\"blend\":" << config.residual_blend << ','
               << "\"min_confidence\":" << config.residual_min_confidence << ','
               << "\"max_delta_x_m_n\":["
               << config.residual_max_delta[0] << ','
               << config.residual_max_delta[1] << ','
               << config.residual_max_delta[2] << "],"
               << "\"max_rate_x_m_n\":["
               << config.residual_max_rate[0] << ','
               << config.residual_max_rate[1] << ','
               << config.residual_max_rate[2] << "]},"
               << "\"ekf_accel\":\"" << accel_mode_name(accel_mode) << "\","
               << "\"allow_truth_heading_aid\":"
               << (config.allow_truth_heading_aid ? "true" : "false") << ','
               << "\"publish_truth_state\":"
               << (config.publish_truth_state ? "true" : "false") << ','
               << "\"control_feedback_source\":\""
               << control_feedback_source_name(config.control_feedback_source) << "\","
               << "\"truth_logging\":\"20 Hz when HIL_TRUTH_STATE is valid\","
               << "\"coordinate_frame\":\"NED\","
               << "\"units\":\"SI\","
               << "\"notes\":\"XLog 1.0 chunked records with CRC32, schema hash and segmented files\""
               << "}";
        return stream.str();
    }

    bool setpoint_changed(const xlog::HydroxSetpointRecord &current,
                          const xlog::HydroxSetpointRecord &previous)
    {
        return current.depth_ref != previous.depth_ref ||
               current.heading_ref != previous.heading_ref ||
               current.surge_ref != previous.surge_ref ||
               current.yaw_rate_ref != previous.yaw_rate_ref ||
               current.wp_n != previous.wp_n ||
               current.wp_e != previous.wp_e ||
               current.wp_d != previous.wp_d ||
               current.use_yaw_rate_ref != previous.use_yaw_rate_ref ||
               current.use_path_segment != previous.use_path_segment ||
               current.hold_heading != previous.hold_heading ||
               current.path_start_n != previous.path_start_n ||
               current.path_start_e != previous.path_start_e ||
               current.lookahead_m != previous.lookahead_m ||
               current.arrival_radius_m != previous.arrival_radius_m;
    }
}

XLogRecorder::XLogRecorder(const Config &config,
                           const ControlParameters &vehicle_params,
                           AccelMode accel_mode,
                           int effective_rate_hz,
                           const XLogIdentity &identity)
    : config_(config),
      vehicle_params_(vehicle_params),
      accel_mode_(accel_mode),
      identity_(identity),
      effective_rate_hz_(std::max(1, effective_rate_hz)),
      active_actuator_count_(active_actuator_count(vehicle_params)),
      estimator_stride_(static_cast<uint32_t>(std::max(1, effective_rate_hz_ / 20))),
      truth_stride_(estimator_stride_)
{
    const std::filesystem::path path =
        xlog_auto(config.xlog) ? default_path(config) : std::filesystem::path(config.xlog);
    if (open_for_session(identity.recording_session_id, path.string()))
    {
        std::printf("[FC] XLog:    %s\n", active_xlog_path_.c_str());
        return;
    }
}

XLogRecorder::~XLogRecorder() = default;

bool XLogRecorder::open_for_session(const std::string&, const std::string &path)
{
    active_xlog_path_ = path;
    std::string error;
    if (!writer_.open(path, metadata_json(config_, vehicle_params_, accel_mode_, identity_), &error))
        return false;
    start_session_clock();
    return true;
}

void XLogRecorder::record_safety(const runtime::HilRuntimeTick& tick, uint64_t now_us,
                                runtime::RuntimeEvent event)
{
    const auto& supervisor = tick.safety_status;
    const bool changed = !have_safety_ ||
        last_safety_.transition_sequence != supervisor.transition_sequence ||
        (last_safety_.actuator_authorized != 0) != supervisor.actuator_authorized;
    if (!changed && !first_fault_pending_ && event == runtime::RuntimeEvent::NONE) return;
    xlog::HydroxSafetyEventRecord r;
    r.wall_unix_ns = xlog::unix_time_ns_now(); r.monotonic_us = now_us;
    r.sensor_us = tick.sensor_time_us; r.transition_sequence = supervisor.transition_sequence;
    r.observed_s = tick.safety_observed_s; r.threshold_s = tick.safety_threshold_s;
    r.position_age_s = tick.position_aid_age_s; r.vertical_age_s = tick.vertical_aid_age_s;
    std::snprintf(r.previous_mode, sizeof(r.previous_mode), "%s", have_safety_ ? last_safety_.mode : "NotStarted");
    std::snprintf(r.mode, sizeof(r.mode), "%s", safety::vehicle_mode_name(supervisor.mode));
    std::snprintf(r.action, sizeof(r.action), "%s", safety::safety_action_name(supervisor.action));
    std::snprintf(r.reason, sizeof(r.reason), "%s", safety::safety_reason_name(supervisor.reason));
    std::snprintf(r.source, sizeof(r.source), "%s", safety::control_source_name(tick.safety_control_source));
    std::snprintf(r.cause, sizeof(r.cause), "%s",
        event != runtime::RuntimeEvent::NONE ? runtime::runtime_event_name(event) : tick.safety_cause);
    r.armed = supervisor.armed; r.actuator_authorized = supervisor.actuator_authorized;
    r.ack_required = supervisor.operator_ack_required;
    r.first_fault = !first_fault_recorded_ && supervisor.operator_ack_required;
    r.position_available = tick.position_available; r.vertical_available = tick.vertical_available;
    r.state_finite = tick.control_state.eta.allFinite() && tick.control_state.nu.allFinite() &&
        std::isfinite(tick.control_state.depth_m);
    // Hold the first fault in producer memory if the reserved queue is busy.
    // Later navigation/medium failures must not replace its cause.
    if (r.first_fault && !first_fault_pending_)
    { pending_first_fault_ = r; first_fault_pending_ = true; }
    if (first_fault_pending_)
    {
        if (!writer_.write_critical(xlog::TopicId::HydroxSafetyEvent,
                pending_first_fault_.sensor_us * 1000ULL, pending_first_fault_)) return;
        first_fault_pending_ = false; first_fault_recorded_ = true;
        if (r.transition_sequence == pending_first_fault_.transition_sequence)
        { have_safety_ = true; last_safety_ = r; writer_.flush(); return; }
        r.first_fault = 0;
    }
    if (writer_.write_critical(xlog::TopicId::HydroxSafetyEvent, r.sensor_us * 1000ULL, r))
    {
        first_fault_recorded_ |= r.first_fault != 0;
        have_safety_ = true; last_safety_ = r;
        writer_.flush(); // Asynchronous request, including IMU-silent faults.
    }
}

void XLogRecorder::start_session_clock()
{
    wall_start_ = std::chrono::steady_clock::now();
}

void XLogRecorder::record_tick(const XLogTickData &data)
{
    if (!writer_.is_open())
        return;
    if (!data.state || !data.setpoint || !data.wrench || !data.actuator ||
        !data.navigation || !data.ekf)
    {
        std::fprintf(stderr, "[FC] ERROR: incomplete XLog tick data; recorder disabled\n");
        active_xlog_path_.clear();
        return;
    }

    const NavigationState &state = *data.state;
    const GNCSetpoint &setpoint = *data.setpoint;
    const Wrench &wrench = *data.wrench;
    const ActuatorCmd &actuator = *data.actuator;
    const NavigationInput &navigation = *data.navigation;
    const auto &channels = actuator.ch;

    const double elapsed_s = wall_start_ == std::chrono::steady_clock::time_point{}
                                 ? 0.0
                                 : std::chrono::duration<double>(data.wall_time - wall_start_).count();
    const uint64_t timestamp_ns = navigation.imu.time_usec > 0
                                      ? navigation.imu.time_usec * 1000ULL
                                      : static_cast<uint64_t>(std::max(0.0, elapsed_s) * 1.0e9);
    const double setpoint_age_s = data.have_external_setpoint
                                      ? std::max(0.0, data.setpoint_age_s)
                                      : -1.0;

    const int active_count = std::max(
        1, std::min<int>(active_actuator_count_, static_cast<int>(channels.size())));
    double max_abs_actuator = 0.0;
    int saturated_count = 0;
    for (int i = 0; i < active_count; ++i)
    {
        const double magnitude = std::abs(static_cast<double>(channels[i]));
        max_abs_actuator = std::max(max_abs_actuator, magnitude);
        if (magnitude >= 0.98)
            ++saturated_count;
    }

    bool ok = true;
    xlog::HydroxStateRecord state_record;
    for (int i = 0; i < 6; ++i)
    {
        state_record.eta[i] = state.eta[i];
        state_record.nu[i] = state.nu[i];
    }
    state_record.depth_m = state.depth_m;
    state_record.gnc_mode = static_cast<uint8_t>(data.gnc_mode);
    state_record.mission_state = data.mission_state;
    state_record.dvl_valid = state.dvl_valid ? 1u : 0u;
    state_record.ekf_init = data.ekf_initialized ? 1u : 0u;
    state_record.controller_reset = data.controller_reset ? 1u : 0u;
    state_record.actuator_authorized = data.actuator_authorized ? 1u : 0u;
    state_record.used_truth = data.used_truth ? 1u : 0u;

    xlog::HydroxSetpointRecord setpoint_record;
    setpoint_record.depth_ref = setpoint.depth_ref;
    setpoint_record.heading_ref = setpoint.heading_ref;
    setpoint_record.surge_ref = setpoint.surge_ref;
    setpoint_record.yaw_rate_ref = setpoint.yaw_rate_ref;
    setpoint_record.wp_n = setpoint.wp_n;
    setpoint_record.wp_e = setpoint.wp_e;
    setpoint_record.wp_d = setpoint.wp_d;
    setpoint_record.setpoint_age_s = setpoint_age_s;
    setpoint_record.use_yaw_rate_ref = setpoint.use_yaw_rate_ref ? 1u : 0u;
    setpoint_record.use_path_segment = setpoint.use_path_segment ? 1u : 0u;
    setpoint_record.hold_heading = setpoint.hold_heading ? 1u : 0u;
    setpoint_record.path_start_n = setpoint.path_start_n;
    setpoint_record.path_start_e = setpoint.path_start_e;
    setpoint_record.lookahead_m = setpoint.lookahead_m;
    setpoint_record.arrival_radius_m = setpoint.arrival_radius_m;

    const bool heartbeat_due =
        !have_last_setpoint_ ||
        timestamp_ns < last_setpoint_timestamp_ns_ ||
        timestamp_ns - last_setpoint_timestamp_ns_ >= kSetpointHeartbeatPeriodNs;
    if (!have_last_setpoint_ ||
        setpoint_changed(setpoint_record, last_setpoint_) || heartbeat_due)
    {
        ok = writer_.write(xlog::TopicId::HydroxSetpoint, timestamp_ns, setpoint_record);
        if (ok)
        {
            last_setpoint_ = setpoint_record;
            last_setpoint_timestamp_ns_ = timestamp_ns;
            have_last_setpoint_ = true;
        }
    }

    xlog::HydroxControlErrorRecord error_record;
    error_record.depth_err = setpoint.depth_ref - state.depth_m;
    error_record.heading_err = wrap_pi(setpoint.heading_ref - state.eta[5]);
    error_record.surge_err = setpoint.surge_ref - state.nu[0];
    error_record.yaw_rate_err =
        setpoint.use_yaw_rate_ref ? setpoint.yaw_rate_ref - state.nu[5] : 0.0;
    error_record.wp_dist = data.waypoint_distance_m;
    if (ok)
        ok = writer_.write(xlog::TopicId::HydroxControlError, timestamp_ns, error_record);

    xlog::HydroxControllerOutputRecord wrench_record;
    for (int i = 0; i < 6; ++i)
        wrench_record.tau[i] = wrench[i];
    wrench_record.tau_norm = wrench.norm();
    if (ok)
        ok = writer_.write(xlog::TopicId::HydroxControllerOutput, timestamp_ns, wrench_record);

    if (ok && data.residual_policy != nullptr)
    {
        ok = writer_.write(
            xlog::TopicId::HydroxResidualPolicy,
            timestamp_ns,
            *data.residual_policy);
    }

    xlog::HydroxActuatorRecord actuator_record;
    for (std::size_t i = 0; i < channels.size(); ++i)
        actuator_record.ch[i] = channels[i];
    actuator_record.rpm = actuator.rpm;
    actuator_record.thrust = channels.size() > 4 ? static_cast<double>(channels[4]) : 0.0;
    actuator_record.max_abs_actuator = max_abs_actuator;
    actuator_record.actuator_sat_ratio =
        static_cast<double>(saturated_count) / static_cast<double>(active_count);
    actuator_record.active_count = static_cast<uint8_t>(active_count);
    if (ok)
        ok = writer_.write(xlog::TopicId::HydroxActuator, timestamp_ns, actuator_record);

    if (ok && data.tick % estimator_stride_ == 0)
    {
        const auto& m = navigation.measurements;
        const auto& stats = data.ekf->last_stats();
        xlog::HydroxNavigationHealthRecord health;
        health.depth_age_s = m.depth.meta.age_s;
        health.gps_age_s = m.gps_position_ned.meta.age_s;
        health.airspeed_age_s = m.airspeed.meta.age_s;
        health.depth_m = m.depth.value; health.airspeed_mps = m.airspeed.value;
        health.depth_accepted = stats.depth_accepted; health.depth_rejected = stats.depth_rejected;
        health.gps_z_accepted = stats.gps_z_accepted; health.gps_z_rejected = stats.gps_z_rejected;
        health.depth_valid = m.depth.meta.valid; health.gps_valid = m.gps_position_ned.meta.valid;
        health.airspeed_valid = m.airspeed.meta.valid; health.imu_valid = navigation.got_imu;
        health.depth_source = static_cast<uint8_t>(m.depth.meta.source);
        ok = writer_.write(xlog::TopicId::HydroxNavigationHealth, timestamp_ns, health);
        const auto &covariance = data.ekf->covariance();
        double pose_covariance_trace = 0.0;
        for (int i = 0; i < 6; ++i)
            pose_covariance_trace += covariance(i, i);

        xlog::HydroxEstimatorHealthRecord estimator_record;
        estimator_record.dvl_age_s = navigation.dvl_age_s;
        estimator_record.accel_norm = navigation.accel_body.norm();
        estimator_record.gyro_norm = navigation.omega_body.norm();
        estimator_record.pose_cov_trace = pose_covariance_trace;
        estimator_record.twist_cov_trace =
            covariance(6, 6) + covariance(7, 7) + covariance(8, 8) + 0.03;
        estimator_record.ekf_have_accel = navigation.have_accel ? 1u : 0u;
        estimator_record.dvl_valid = state.dvl_valid ? 1u : 0u;
        estimator_record.gps_valid = data.gps_valid ? 1u : 0u;
        ok = writer_.write(
            xlog::TopicId::HydroxEstimatorHealth, timestamp_ns, estimator_record);
    }

    xlog::HydroxTimingRecord timing_record;
    timing_record.dt = data.dt;
    timing_record.expected_dt = data.expected_dt;
    timing_record.setpoint_age_s = setpoint_age_s;
    timing_record.loop_overrun = data.dt > data.expected_dt * 1.5 ? 1u : 0u;
    if (ok)
        ok = writer_.write(xlog::TopicId::HydroxTiming, timestamp_ns, timing_record);

    if (ok && navigation.truth_valid && data.tick % truth_stride_ == 0)
    {
        xlog::SimulatorTruthRecord truth_record;
        for (int i = 0; i < 6; ++i)
        {
            truth_record.eta[i] = navigation.truth.eta[i];
            truth_record.nu[i] = navigation.truth.nu[i];
        }
        truth_record.source_timestamp_us = navigation.truth.time_usec;
        truth_record.valid = 1u;
        ok = writer_.write(xlog::TopicId::SimulatorTruth, timestamp_ns, truth_record);
    }

    // State remains last so replay consumers first see the matching control data.
    if (ok)
        ok = writer_.write(xlog::TopicId::HydroxState, timestamp_ns, state_record);
    if (ok && data.tick % static_cast<uint32_t>(effective_rate_hz_) == 0)
        ok = writer_.flush();

    if (!ok)
    {
        std::fprintf(stderr,
                     "[FC] ERROR: XLog writer failed and has been disabled: %s\n",
                     writer_.last_error().empty()
                         ? "unknown write error"
                         : writer_.last_error().c_str());
    }
}

} // namespace hydrox::sitl
