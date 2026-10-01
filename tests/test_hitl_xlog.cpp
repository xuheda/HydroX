#include "hitl/hitl_xlog.h"
#include "learning/xlog_control_shadow.h"
#include "xlog_reader.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
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
    namespace fs = std::filesystem;
    const fs::path directory = fs::temp_directory_path() /
        ("hydrox_test_hitl_xlog_" +
         std::to_string(hydrox::xlog::unix_time_ns_now()));
    fs::create_directories(directory);
    const fs::path path = directory / "hitl.xlog";

    hydrox::hitl::XLogConfig config;
    config.path = path.string();
    config.vehicle = "hitl-test-vehicle";
    config.profile_id = "generic-auv-fin";
    config.recording_session_id = "hitl-recording-session";
    config.profile_fingerprint = 0x123456789abcdef0ULL;
    config.session_config.nominal_period_us = 10'000;
    config.session_digest = hydrox::runtime::hil_session_digest(
        config.profile_fingerprint, config.session_config);

    hydrox::hitl::XLogRecorder recorder;
    std::string error;
    if (!recorder.open(config, &error))
        return fail(error.c_str());

    hydrox::HilControlTraceMsg trace;
    trace.valid = true;
    trace.time_usec = 5'000'000;
    trace.tick = 11;
    trace.dt_s = 0.01;
    trace.mode = static_cast<uint8_t>(hydrox::GNCMode::WAYPOINT_3D);
    trace.mission_state = 1;
    trace.actuator_channel_count = 5;
    trace.flags =
        hydrox::HIL_CONTROL_TRACE_FLAG_USE_PATH_SEGMENT |
        hydrox::HIL_CONTROL_TRACE_FLAG_CONTROLLER_RESET |
        hydrox::HIL_CONTROL_TRACE_FLAG_ACTUATOR_AUTHORIZED |
        hydrox::HIL_CONTROL_TRACE_FLAG_EXTERNAL_SETPOINT |
        hydrox::HIL_CONTROL_TRACE_FLAG_DVL_VALID |
        hydrox::HIL_CONTROL_TRACE_FLAG_EKF_INITIALIZED;
    trace.eta[0] = 5.0;
    trace.eta[1] = -2.0;
    trace.eta[2] = 4.0;
    trace.nu[0] = 0.8;
    trace.depth_m = 4.0;
    trace.depth_ref = 7.0;
    trace.heading_ref = 0.3;
    trace.surge_ref = 1.4;
    trace.wp_n = 30.0;
    trace.wp_e = 10.0;
    trace.wp_d = 7.0;
    trace.path_start_n = -5.0;
    trace.path_start_e = -3.0;
    trace.lookahead_m = 8.0;
    trace.arrival_radius_m = 2.0;
    trace.wrench[0] = 25.0f;
    trace.wrench[4] = -4.0f;
    trace.wrench[5] = 3.0f;
    trace.setpoint_age_s = 0.02f;

    hydrox::HilActuatorControlsMsg actuator;
    actuator.valid = true;
    actuator.time_usec = trace.time_usec;
    actuator.controls[0] = 0.1f;
    actuator.controls[4] = 0.6f;
    if (!recorder.record(trace, &actuator))
        return fail("record HITL control trace");
    if (recorder.stats().traces != 1 ||
        recorder.stats().traces_with_actuator != 1 ||
        recorder.stats().traces_without_actuator != 0)
        return fail("HITL recorder statistics");
    recorder.close();

    hydrox::xlog::Reader raw_reader;
    if (!raw_reader.read(path.string(), {}, &error))
        return fail(error.c_str());
    const std::string metadata = raw_reader.metadata_json();
    if (metadata.find("\"producer\":\"hydrox_hitl_router\"") ==
            std::string::npos ||
        metadata.find("\"profile_bound\":true") == std::string::npos ||
        metadata.find("\"recording_session_id\":\"hitl-recording-session\"") ==
            std::string::npos ||
        metadata.find("\"session_config_payload\":\"") ==
            std::string::npos)
        return fail("HITL XLog Profile/Session metadata");

    hydrox::learning::XLogControlShadowReader adapter;
    std::size_t frames = 0;
    if (!adapter.read(
            path.string(),
            [&](const hydrox::learning::XLogControlFrame &frame)
            {
                ++frames;
                return frame.actuator_authorized &&
                    frame.input.reset_controller &&
                    frame.input.setpoint.use_path_segment &&
                    frame.input.setpoint.path_start_n == trace.path_start_n &&
                    frame.input.setpoint.lookahead_m == trace.lookahead_m &&
                    frame.have_recorded_wrench &&
                    frame.recorded_wrench.tau[0] == trace.wrench[0] &&
                    frame.have_recorded_actuator &&
                    std::abs(frame.recorded_actuator.ch[4] -
                             actuator.controls[4]) < 1.0e-6f;
            },
            &error))
        return fail(error.c_str());
    if (frames != 1 || adapter.stats().emitted_frames != 1)
        return fail("HITL XLog control frame assembly");

    std::error_code cleanup_error;
    fs::remove_all(directory, cleanup_error);
    std::puts("PASS: HITL control trace to XLog adapter");
    return 0;
}
