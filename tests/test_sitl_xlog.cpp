// Copyright (c) 2026 OceanX. Author: xuheda
#include "sitl/sitl_xlog.h"
#include "vehicle_bundle.h"
#include "xlog_reader.h"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>

namespace
{
    int fail(const char *message)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        return 1;
    }
}

int main()
{
    using namespace hydrox;

    const std::filesystem::path directory =
        std::filesystem::temp_directory_path() /
        ("hydrox_test_sitl_xlog_" +
         std::to_string(xlog::unix_time_ns_now()));
    const std::filesystem::path path = directory / "sitl_recording.xlog";

    sitl::Config config;
    config.vehicle = "test_vehicle";
    config.vehicle_type = "EcaA9";
    config.xlog = path.string();
    config.control_feedback_source = runtime::ControlFeedbackSource::TruthDebug;
    config.residual_policy_mode = sitl::ResidualPolicyMode::Shadow;
    config.residual_policy_port = 14750;
    config.residual_policy_local_port = 14751;
    config.residual_policy_nonce = 20260921;

    const ControlParameters params = load_vehicle_bundle("profiles/eca-a9/vehicle-bundle.json").control;
    if (!params.valid)
        return fail("built-in vehicle parameters");

    NavigationState state = NavigationState::zeros();
    state.depth_m = 5.0;
    state.dvl_valid = true;

    GNCSetpoint setpoint;
    setpoint.depth_ref = 5.0;
    setpoint.heading_ref = 0.25;
    setpoint.surge_ref = 1.0;
    setpoint.use_path_segment = true;
    setpoint.path_start_n = -3.0;
    setpoint.path_start_e = 2.0;
    setpoint.lookahead_m = 8.0;
    setpoint.arrival_radius_m = 2.5;

    Wrench wrench = Wrench::Zero();
    wrench[0] = 12.0;
    ActuatorCmd actuator;
    actuator.layout = ActuatorLayout::DirectBodyWrench;
    actuator.set_body_force(0, 2.0, 20.0);
    actuator.set_body_force(1, 0.0, 20.0);
    actuator.set_body_force(2, 0.0, 20.0);
    actuator.set_body_moment(3, 0.0, 20.0);
    actuator.set_body_moment(4, 4.0, 20.0);
    actuator.rpm = 900.0;
    actuator.set_body_moment(5, 0.0, 20.0);

    xlog::HydroxResidualPolicyRecord residual_policy;
    residual_policy.base_tau[0] = 12.0;
    residual_policy.candidate_tau[0] = 13.5;
    residual_policy.applied_tau[0] = 12.0;
    residual_policy.inference_ms = 3.5;
    residual_policy.pinn_ood_feature = 0.02;
    residual_policy.selector_fraction_x_n[0] = 0.75;
    residual_policy.selector_fraction_x_n[1] = 0.5;
    residual_policy.requests = 4;
    residual_policy.accepted = 3;
    residual_policy.timeouts = 1;
    residual_policy.observed = 1;
    residual_policy.candidate_valid = 1;
    residual_policy.mode = 1;

    NavigationInput navigation;
    navigation.imu.time_usec = 1'000'000ULL;
    navigation.have_accel = true;
    navigation.accel_body = Eigen::Vector3d(0.0, 0.0, -EKF_G);
    navigation.omega_body = Eigen::Vector3d::Zero();
    navigation.truth_valid = true;
    navigation.truth.time_usec = navigation.imu.time_usec;
    navigation.truth.valid = true;
    for (int i = 0; i < 6; ++i)
    {
        navigation.truth.eta[i] = state.eta[i];
        navigation.truth.nu[i] = state.nu[i];
    }

    EKF ekf;
    ekf.reset(state);

    sitl::XLogIdentity identity;
    identity.profile_id = "test-profile";
    identity.control_contract = "test-contract-v1";
    identity.profile_fingerprint = 0x1122334455667788ULL;
    identity.profile_bound = true;
    identity.session_config.nominal_period_us = 10'000;
    identity.session_digest = runtime::hil_session_digest(
        identity.profile_fingerprint, identity.session_config);

    {
        sitl::XLogRecorder recorder(
            config, params, AccelMode::Auto, 100, identity);
        recorder.start_session_clock();

        sitl::XLogTickData tick;
        tick.state = &state;
        tick.setpoint = &setpoint;
        tick.wrench = &wrench;
        tick.actuator = &actuator;
        tick.residual_policy = &residual_policy;
        tick.navigation = &navigation;
        tick.ekf = &ekf;
        tick.tick = 100;
        tick.gnc_mode = GNCMode::DEPTH_HOLD;
        tick.mission_state = 1;
        tick.gps_valid = false;
        tick.ekf_initialized = true;
        tick.controller_reset = true;
        tick.actuator_authorized = true;
        tick.used_truth = true;
        tick.dt = 0.01;
        tick.expected_dt = 0.01;
        tick.wall_time = std::chrono::steady_clock::now();
        recorder.record_tick(tick);
    }

    std::ifstream input(path, std::ios::binary);
    xlog::FileHeader header{};
    input.read(reinterpret_cast<char *>(&header), sizeof(header));
    if (!input || std::memcmp(header.magic, "XLOG", 4) != 0)
        return fail("recorded XLog header");

    std::string metadata(
        static_cast<std::size_t>(header.metadata_len),
        '\0');
    input.read(metadata.data(), static_cast<std::streamsize>(metadata.size()));
    if (!input ||
        metadata.find("\"control_feedback_source\":\"truth_debug\"") ==
            std::string::npos)
    {
        return fail("control feedback source recorded in XLog metadata");
    }
    if (metadata.find("\"publish_truth_state\":false") ==
            std::string::npos)
    {
        return fail("DDS truth publication setting recorded in XLog metadata");
    }
    if (metadata.find("\"residual_policy\":{\"mode\":\"SHADOW\"") ==
            std::string::npos ||
        metadata.find("\"nonce\":20260921") == std::string::npos)
    {
        return fail("residual policy contract recorded in XLog metadata");
    }
    if (metadata.find("\"profile_id\":\"test-profile\"") == std::string::npos ||
        metadata.find("\"profile_fingerprint\":\"1122334455667788\"") == std::string::npos ||
        metadata.find("\"profile_bound\":true") == std::string::npos ||
        metadata.find("\"session_digest\":\"") == std::string::npos ||
        metadata.find("\"session_config_payload\":\"") == std::string::npos ||
        metadata.find("\"nominal_period_us\":10000") == std::string::npos)
    {
        return fail("Profile and Session identity recorded in XLog metadata");
    }
    input.close();

    bool saw_setpoint_v2 = false;
    bool saw_state_v2 = false;
    bool saw_residual_policy = false;
    xlog::Reader reader;
    std::string reader_error;
    if (!reader.read(
        path.string(),
        [&](const xlog::RecordView &view)
        {
            if (view.header.topic_id ==
                    static_cast<uint16_t>(xlog::TopicId::HydroxSetpoint) &&
                view.header.payload_size == sizeof(xlog::HydroxSetpointRecord))
            {
                xlog::HydroxSetpointRecord record{};
                std::memcpy(&record, view.payload, sizeof(record));
                saw_setpoint_v2 = record.use_path_segment != 0 &&
                    record.path_start_n == setpoint.path_start_n &&
                    record.path_start_e == setpoint.path_start_e &&
                    record.lookahead_m == setpoint.lookahead_m &&
                    record.arrival_radius_m == setpoint.arrival_radius_m;
            }
            if (view.header.topic_id ==
                    static_cast<uint16_t>(xlog::TopicId::HydroxState) &&
                view.header.payload_size == sizeof(xlog::HydroxStateRecord))
            {
                xlog::HydroxStateRecord record{};
                std::memcpy(&record, view.payload, sizeof(record));
                saw_state_v2 = record.controller_reset != 0 &&
                    record.actuator_authorized != 0 && record.used_truth != 0;
            }
            if (view.header.topic_id ==
                    static_cast<uint16_t>(xlog::TopicId::HydroxResidualPolicy) &&
                view.header.payload_size ==
                    sizeof(xlog::HydroxResidualPolicyRecord))
            {
                xlog::HydroxResidualPolicyRecord record{};
                std::memcpy(&record, view.payload, sizeof(record));
                saw_residual_policy =
                    record.base_tau[0] == 12.0 &&
                    record.candidate_tau[0] == 13.5 &&
                    record.applied_tau[0] == 12.0 &&
                    record.requests == 4 && record.accepted == 3 &&
                    record.timeouts == 1 && record.observed != 0 &&
                    record.candidate_valid != 0 && record.active == 0 &&
                    record.mode == 1;
            }
            return true;
        },
        &reader_error))
    {
        return fail(reader_error.c_str());
    }
    if (!saw_setpoint_v2 || !saw_state_v2 || !saw_residual_policy)
        return fail("complete control replay fields recorded");

    if (std::filesystem::file_size(path) <= sizeof(xlog::FileHeader))
        return fail("recorded XLog payload");

    // Default diagnostics require neither a UI recording command nor IMU data.
    const auto automatic_dir = directory / "automatic";
    auto automatic = config;
    automatic.xlog = "auto";
    automatic.log_directory = automatic_dir.string();
    automatic.run_id = "test-process-run";
    {
        sitl::XLogRecorder recorder(automatic, params, AccelMode::Auto, 100, identity);
        runtime::HilRuntimeTick safety_tick;
        safety_tick.safety_status.transition_sequence = 1;
        safety_tick.safety_status.armed = true;
        safety_tick.safety_status.operator_ack_required = true;
        safety_tick.safety_status.mode = safety::VehicleMode::OutputDisabled;
        safety_tick.safety_status.reason = safety::SafetyReason::ControlCoreUnavailable;
        safety_tick.safety_status.action = safety::SafetyAction::DisableOutput;
        safety_tick.safety_cause = "SENSOR_TIMEOUT";
        safety_tick.safety_observed_s = .62;
        safety_tick.safety_threshold_s = .5;
        recorder.record_safety(safety_tick, 620000, runtime::RuntimeEvent::SENSOR_TIMEOUT);
        safety_tick.safety_status.transition_sequence++;
        safety_tick.safety_cause = "VerticalAidUnavailable";
        recorder.record_safety(safety_tick, 630000);
    }
    int events = 0, first_faults = 0;
    for (const auto& entry : std::filesystem::directory_iterator(automatic_dir))
    {
        if (entry.path().extension() != ".xlog") continue;
        xlog::Reader diagnostic_reader;
        if (!diagnostic_reader.read(entry.path().string(), [&](const xlog::RecordView& view) {
            if (view.header.topic_id == static_cast<uint16_t>(xlog::TopicId::HydroxSafetyEvent))
            {
                xlog::HydroxSafetyEventRecord event;
                std::memcpy(&event, view.payload, sizeof(event));
                ++events; first_faults += event.first_fault;
                if (event.first_fault && (std::string(event.cause) != "SENSOR_TIMEOUT" ||
                    event.observed_s != .62 || event.threshold_s != .5))
                    first_faults += 100;
            }
            return true;
        }, &reader_error)) return fail(reader_error.c_str());
    }
    if (events != 2 || first_faults != 1) return fail("IMU-silent first fault persists without UI recording");

    std::error_code cleanup_error;
    std::filesystem::remove_all(directory, cleanup_error);
    std::printf("test_sitl_xlog: all checks passed\n");
    return 0;
}
