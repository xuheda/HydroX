#include "hitl/hitl_xlog.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace hydrox::hitl
{
namespace
{
    template<typename Bytes>
    std::string bytes_hex(const Bytes &bytes)
    {
        std::ostringstream stream;
        stream << std::hex << std::setfill('0');
        for (const uint8_t byte : bytes)
            stream << std::setw(2) << static_cast<unsigned int>(byte);
        return stream.str();
    }

    std::string hex_u64(uint64_t value)
    {
        std::ostringstream stream;
        stream << std::hex << std::setw(16) << std::setfill('0') << value;
        return stream.str();
    }

    std::string metadata_json(const XLogConfig &config)
    {
        const runtime::HilSessionConfigPayload payload =
            runtime::encode_hil_session_config(config.session_config);
        const runtime::HilSessionConfigV1 &session = config.session_config;
        std::ostringstream stream;
        stream << '{'
               << "\"format\":\"XLog\","
               << "\"format_version\":\"1.0\","
               << "\"producer\":\"hydrox_hitl_router\","
               << "\"capture_source\":\"fmuv6c_hil_control_trace\","
               << "\"control_trace_contract\":\"hydrox.hil/control-trace@1\","
               << "\"vehicle\":\"" << xlog::json_escape(config.vehicle) << "\","
               << "\"profile_id\":\"" << xlog::json_escape(config.profile_id) << "\","
               << "\"recording_session_id\":\""
               << xlog::json_escape(config.recording_session_id) << "\","
               << "\"profile_bound\":true,"
               << "\"profile_fingerprint\":\""
               << hex_u64(config.profile_fingerprint) << "\","
               << "\"session_digest\":\""
               << bytes_hex(config.session_digest) << "\","
               << "\"session_config_payload\":\""
               << bytes_hex(payload) << "\","
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
               << "\"feedback_source\":"
               << static_cast<unsigned int>(session.feedback_source)
               << "},\"coordinate_frame\":\"NED\",\"units\":\"SI\"}";
        return stream.str();
    }
}

bool XLogRecorder::open(const XLogConfig &config, std::string *error)
{
    close();
    stats_ = {};
    last_tick_ = 0;
    runtime::HilSessionField field = runtime::HilSessionField::None;
    if (config.path.empty() || config.vehicle.empty() ||
        config.profile_id.empty() || config.profile_fingerprint == 0 ||
        !runtime::validate_hil_session_config(config.session_config, field) ||
        runtime::hil_session_digest_is_zero(config.session_digest) ||
        !runtime::hil_session_digest_equal(
            config.session_digest,
            runtime::hil_session_digest(
                config.profile_fingerprint, config.session_config)))
    {
        if (error != nullptr)
            *error = "invalid Profile-bound HITL XLog configuration";
        return false;
    }
    expected_dt_s_ = static_cast<double>(
        config.session_config.nominal_period_us) * 1.0e-6;
    flush_stride_ = std::max<uint32_t>(
        1, static_cast<uint32_t>(std::llround(1.0 / expected_dt_s_)));
    return writer_.open(config.path, metadata_json(config), error);
}

bool XLogRecorder::record(const HilControlTraceMsg &trace,
                          const HilActuatorControlsMsg *actuator)
{
    if (!writer_.is_open() || !trace.valid)
        return false;

    const uint64_t timestamp_ns = trace.time_usec * 1000ULL;
    if (last_tick_ != 0 && trace.tick > last_tick_ + 1)
        stats_.missing_trace_ticks += trace.tick - last_tick_ - 1;
    last_tick_ = trace.tick;

    xlog::HydroxSetpointRecord setpoint;
    setpoint.depth_ref = trace.depth_ref;
    setpoint.heading_ref = trace.heading_ref;
    setpoint.surge_ref = trace.surge_ref;
    setpoint.yaw_rate_ref = trace.yaw_rate_ref;
    setpoint.wp_n = trace.wp_n;
    setpoint.wp_e = trace.wp_e;
    setpoint.wp_d = trace.wp_d;
    setpoint.setpoint_age_s = trace.setpoint_age_s;
    setpoint.use_yaw_rate_ref =
        (trace.flags & HIL_CONTROL_TRACE_FLAG_USE_YAW_RATE) != 0;
    setpoint.use_path_segment =
        (trace.flags & HIL_CONTROL_TRACE_FLAG_USE_PATH_SEGMENT) != 0;
    setpoint.hold_heading =
        (trace.flags & HIL_CONTROL_TRACE_FLAG_HOLD_HEADING) != 0;
    setpoint.path_start_n = trace.path_start_n;
    setpoint.path_start_e = trace.path_start_e;
    setpoint.lookahead_m = trace.lookahead_m;
    setpoint.arrival_radius_m = trace.arrival_radius_m;
    if (!writer_.write(xlog::TopicId::HydroxSetpoint, timestamp_ns, setpoint))
        return false;

    xlog::HydroxTimingRecord timing;
    timing.dt = trace.dt_s;
    timing.expected_dt = expected_dt_s_;
    timing.setpoint_age_s = trace.setpoint_age_s;
    timing.loop_overrun = trace.dt_s > expected_dt_s_ * 1.5 ? 1u : 0u;
    if (!writer_.write(xlog::TopicId::HydroxTiming, timestamp_ns, timing))
        return false;

    xlog::HydroxControllerOutputRecord wrench;
    double tau_norm_squared = 0.0;
    for (std::size_t index = 0; index < 6; ++index)
    {
        wrench.tau[index] = static_cast<double>(trace.wrench[index]);
        tau_norm_squared += wrench.tau[index] * wrench.tau[index];
    }
    wrench.tau_norm = std::sqrt(tau_norm_squared);
    if (!writer_.write(
            xlog::TopicId::HydroxControllerOutput, timestamp_ns, wrench))
        return false;

    const bool matching_actuator = actuator != nullptr && actuator->valid &&
        actuator->time_usec == trace.time_usec;
    if (matching_actuator)
    {
        xlog::HydroxActuatorRecord output;
        const std::size_t count = std::min<std::size_t>(
            trace.actuator_channel_count, 8);
        double max_abs = 0.0;
        std::size_t saturated = 0;
        for (std::size_t index = 0; index < 8; ++index)
        {
            output.ch[index] = actuator->controls[index];
            if (index < count)
            {
                const double magnitude = std::abs(
                    static_cast<double>(actuator->controls[index]));
                max_abs = std::max(max_abs, magnitude);
                if (magnitude >= 0.98)
                    ++saturated;
            }
        }
        output.max_abs_actuator = max_abs;
        output.actuator_sat_ratio = count > 0
            ? static_cast<double>(saturated) / static_cast<double>(count)
            : 0.0;
        output.active_count = static_cast<uint8_t>(count);
        if (!writer_.write(xlog::TopicId::HydroxActuator, timestamp_ns, output))
            return false;
        ++stats_.traces_with_actuator;
    }
    else
    {
        ++stats_.traces_without_actuator;
    }

    xlog::HydroxStateRecord state;
    for (std::size_t index = 0; index < 6; ++index)
    {
        state.eta[index] = trace.eta[index];
        state.nu[index] = trace.nu[index];
    }
    state.depth_m = trace.depth_m;
    state.gnc_mode = trace.mode;
    state.mission_state = trace.mission_state;
    state.dvl_valid =
        (trace.flags & HIL_CONTROL_TRACE_FLAG_DVL_VALID) != 0;
    state.ekf_init =
        (trace.flags & HIL_CONTROL_TRACE_FLAG_EKF_INITIALIZED) != 0;
    state.controller_reset =
        (trace.flags & HIL_CONTROL_TRACE_FLAG_CONTROLLER_RESET) != 0;
    state.actuator_authorized =
        (trace.flags & HIL_CONTROL_TRACE_FLAG_ACTUATOR_AUTHORIZED) != 0;
    state.used_truth =
        (trace.flags & HIL_CONTROL_TRACE_FLAG_USED_TRUTH) != 0;
    if (!writer_.write(xlog::TopicId::HydroxState, timestamp_ns, state))
        return false;

    ++stats_.traces;
    return stats_.traces % flush_stride_ != 0 || writer_.flush();
}

void XLogRecorder::close()
{
    writer_.close();
}
} // namespace hydrox::hitl
