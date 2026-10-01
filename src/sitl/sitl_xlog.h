#pragma once

#include "ekf.h"
#include "control_parameters.h"
#include "gnc/control_interfaces.h"
#include "hydrox/runtime/hil_session_config.h"
#include "sensor_adapter.h"
#include "sitl_config.h"
#include "xlog_async_writer.h"
#include "hydrox/runtime/hil_runtime.h"

#include <chrono>
#include <cstdint>
#include <string>

namespace hydrox::sitl
{
    struct XLogIdentity
    {
        std::string profile_id;
        std::string control_contract;
        uint64_t profile_fingerprint = 0;
        bool profile_bound = false;
        runtime::HilSessionConfigV1 session_config{};
        runtime::HilSessionDigest session_digest{};
        /** Shared OceanX recording session ID; empty for standalone logging. */
        std::string recording_session_id;
    };

    struct XLogTickData
    {
        const NavigationState *state = nullptr;
        const GNCSetpoint *setpoint = nullptr;
        const Wrench *wrench = nullptr;
        const ActuatorCmd *actuator = nullptr;
        const xlog::HydroxResidualPolicyRecord *residual_policy = nullptr;
        const NavigationInput *navigation = nullptr;
        const EKF *ekf = nullptr;

        uint32_t tick = 0;
        GNCMode gnc_mode = GNCMode::DISABLED;
        uint8_t mission_state = 0;
        bool gps_valid = false;
        bool ekf_initialized = false;
        bool controller_reset = false;
        bool actuator_authorized = false;
        bool used_truth = false;
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
                     const ControlParameters &vehicle_params,
                     AccelMode accel_mode,
                     int effective_rate_hz,
                     const XLogIdentity &identity = {});
        ~XLogRecorder();

        XLogRecorder(const XLogRecorder &) = delete;
        XLogRecorder &operator=(const XLogRecorder &) = delete;

        void start_session_clock();
        void record_safety(const runtime::HilRuntimeTick& tick, uint64_t now_us,
                           runtime::RuntimeEvent event = runtime::RuntimeEvent::NONE);
        void record_tick(const XLogTickData &data);

    private:
        bool open_for_session(const std::string &session_id,
                              const std::string &path);
        xlog::AsyncWriter writer_;
        Config config_;
        ControlParameters vehicle_params_;
        AccelMode accel_mode_ = AccelMode::Auto;
        XLogIdentity identity_;
        std::string active_xlog_path_;
        bool have_safety_ = false, first_fault_recorded_ = false;
        xlog::HydroxSafetyEventRecord last_safety_{}, pending_first_fault_{};
        bool first_fault_pending_ = false;
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
