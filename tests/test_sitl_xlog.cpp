// Copyright (c) 2026 OceanX. Author: xuheda
#include "sitl/sitl_xlog.h"

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

    const FossenControlParams params = builtin_fossen_control_params("EcaA9");
    if (!params.valid)
        return fail("built-in vehicle parameters");

    AUVState state = AUVState::zeros();
    state.depth_m = 5.0;
    state.dvl_valid = true;

    GNCSetpoint setpoint;
    setpoint.depth_ref = 5.0;
    setpoint.heading_ref = 0.25;
    setpoint.surge_ref = 1.0;

    Wrench wrench = Wrench::Zero();
    wrench[0] = 12.0;
    ActuatorCmd actuator;
    actuator.ch[0] = 0.1f;
    actuator.ch[4] = 0.2f;
    actuator.rpm = 900.0;

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

    {
        sitl::XLogRecorder recorder(config, params, AccelMode::Auto, 100);
        recorder.start_session_clock();

        sitl::XLogTickData tick;
        tick.state = &state;
        tick.setpoint = &setpoint;
        tick.wrench = &wrench;
        tick.actuator = &actuator;
        tick.navigation = &navigation;
        tick.ekf = &ekf;
        tick.tick = 100;
        tick.gnc_mode = GNCMode::DEPTH_HOLD;
        tick.mission_state = 1;
        tick.gps_valid = false;
        tick.ekf_initialized = true;
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
    input.close();

    if (std::filesystem::file_size(path) <= sizeof(xlog::FileHeader))
        return fail("recorded XLog payload");

    std::error_code cleanup_error;
    std::filesystem::remove_all(directory, cleanup_error);
    std::printf("test_sitl_xlog: all checks passed\n");
    return 0;
}
