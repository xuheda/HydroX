#pragma once

#include "ekf.h"
#include "fossen_vehicle_params.h"
#include "gnc/control_interfaces.h"
#include "sensor_adapter.h"
#include "sitl_config.h"
#include "xlog_writer.h"

#include <chrono>
#include <cstdint>

namespace hydrox::sitl
{
    struct XLogTickData
    {
        const AUVState *state = nullptr;
        const GNCSetpoint *setpoint = nullptr;
        const Wrench *wrench = nullptr;
        const ActuatorCmd *actuator = nullptr;
        const NavigationInput *navigation = nullptr;
        const EKF *ekf = nullptr;

        uint32_t tick = 0;
        GNCMode gnc_mode = GNCMode::DISABLED;
        uint8_t mission_state = 0;
        bool gps_valid = false;
        bool ekf_initialized = false;
        bool have_external_setpoint = false;
        double setpoint_age_s = -1.0;
        double waypoint_distance_m = -1.0;
        double dt = 0.0;
        double expected_dt = 0.0;
        std::chrono::steady_clock::time_point wall_time{};
    };

    class XLogRecorder
    {
    public:
        XLogRecorder(const Config &config,
                     const FossenControlParams &vehicle_params,
                     AccelMode accel_mode,
                     int effective_rate_hz);

        XLogRecorder(const XLogRecorder &) = delete;
        XLogRecorder &operator=(const XLogRecorder &) = delete;

        void start_session_clock();
        void record_tick(const XLogTickData &data);

    private:
        xlog::Writer writer_;
        int effective_rate_hz_ = 100;
        int active_actuator_count_ = 1;
        uint32_t estimator_stride_ = 1;
        uint32_t truth_stride_ = 1;
        xlog::HydroxSetpointRecord last_setpoint_{};
        bool have_last_setpoint_ = false;
        uint64_t last_setpoint_timestamp_ns_ = 0;
        std::chrono::steady_clock::time_point wall_start_{};
    };

} // namespace hydrox::sitl
