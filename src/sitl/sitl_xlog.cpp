#include "sitl_xlog.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
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
        return std::filesystem::path("log") /
               ("xlog_" + config.vehicle + "_" + timestamp_tag() + ".xlog");
    }

    bool xlog_disabled(const std::string &value)
    {
        return value == "off" || value == "OFF" || value == "0";
    }

    bool xlog_auto(const std::string &value)
    {
        return value == "auto" || value == "AUTO";
    }

    int active_actuator_count(const FossenControlParams &params)
    {
        switch (params.archetype)
        {
        case VehicleArchetype::Thruster:
            if (params.vehicle_type == "DesistekSaga")
                return 3;
            if (params.vehicle_type == "RexROV2")
                return 6;
            return 8;
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
                              const FossenControlParams &params,
                              AccelMode accel_mode)
    {
        std::ostringstream stream;
        stream << "{"
               << "\"format\":\"XLog\","
               << "\"format_version\":\"1.0\","
               << "\"producer\":\"hydrox_sitl\","
               << "\"vehicle\":\"" << xlog::json_escape(config.vehicle) << "\","
               << "\"vehicle_type\":\"" << xlog::json_escape(config.vehicle_type) << "\","
               << "\"resolved_vehicle_type\":\"" << xlog::json_escape(params.vehicle_type) << "\","
               << "\"vehicle_class\":\"" << vehicle_class_name(params.vehicle_class) << "\","
               << "\"vehicle_archetype\":\"" << vehicle_archetype_name(params.archetype) << "\","
               << "\"params_source\":\"" << xlog::json_escape(params.source_path) << "\","
               << "\"params_loaded_from_json\":" << (params.loaded_from_json ? "true" : "false") << ','
               << "\"ue5_host\":\"" << xlog::json_escape(config.ue5_host) << "\","
               << "\"ue5_port\":" << config.ue5_port << ','
               << "\"dds_host\":\"" << xlog::json_escape(config.dds_host) << "\","
               << "\"dds_port\":" << config.dds_port << ','
               << "\"ros_domain_id\":" << config.ros_domain_id << ','
               << "\"rate_hz\":" << config.rate_hz << ','
               << "\"ekf_accel\":\"" << accel_mode_name(accel_mode) << "\","
               << "\"allow_truth_heading_aid\":"
               << (config.allow_truth_heading_aid ? "true" : "false") << ','
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
               current.use_yaw_rate_ref != previous.use_yaw_rate_ref;
    }
}

XLogRecorder::XLogRecorder(const Config &config,
                           const FossenControlParams &vehicle_params,
                           AccelMode accel_mode,
                           int effective_rate_hz)
    : effective_rate_hz_(std::max(1, effective_rate_hz)),
      active_actuator_count_(active_actuator_count(vehicle_params)),
      estimator_stride_(static_cast<uint32_t>(std::max(1, effective_rate_hz_ / 20))),
      truth_stride_(estimator_stride_)
{
    if (xlog_disabled(config.xlog))
    {
        std::printf("[FC] XLog:    off\n");
        return;
    }

    const std::filesystem::path path =
        xlog_auto(config.xlog) ? default_path(config) : std::filesystem::path(config.xlog);
    std::string error;
    if (writer_.open(path.string(), metadata_json(config, vehicle_params, accel_mode), &error))
    {
        std::printf("[FC] XLog:    %s\n", writer_.path().c_str());
        return;
    }

    std::fprintf(stderr, "[FC] WARNING: failed to open XLog: %s (%s)\n",
                 path.string().c_str(),
                 error.empty() ? "unknown error" : error.c_str());
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
        writer_.close();
        return;
    }

    const AUVState &state = *data.state;
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
        writer_.close();
    }
}

} // namespace hydrox::sitl
