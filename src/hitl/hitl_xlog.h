#pragma once

#include "hydrox/runtime/hil_session_config.h"
#include "mavlink_hil.h"
#include "xlog_async_writer.h"

#include <cstdint>
#include <string>

namespace hydrox::hitl
{
    struct XLogConfig
    {
        std::string path;
        std::string vehicle;
        std::string profile_id;
        std::string recording_session_id;
        uint64_t profile_fingerprint = 0;
        runtime::HilSessionConfigV1 session_config{};
        runtime::HilSessionDigest session_digest{};
    };

    struct XLogStats
    {
        uint64_t traces = 0;
        uint64_t traces_with_actuator = 0;
        uint64_t traces_without_actuator = 0;
        uint64_t missing_trace_ticks = 0;
    };

    /** Host-side conversion of board-owned per-tick trace into XLog 1.0. */
    class XLogRecorder
    {
    public:
        bool open(const XLogConfig &config, std::string *error = nullptr);
        bool record(const HilControlTraceMsg &trace,
                    const HilActuatorControlsMsg *actuator = nullptr);
        void close();

        bool is_open() const noexcept { return writer_.is_open(); }
        const std::string &path() const noexcept { return writer_.path(); }
        std::string last_error() const { return writer_.last_error(); }
        const XLogStats &stats() const noexcept { return stats_; }

    private:
        xlog::AsyncWriter writer_;
        XLogStats stats_{};
        double expected_dt_s_ = 0.01;
        uint32_t flush_stride_ = 100;
        uint32_t last_tick_ = 0;
    };
} // namespace hydrox::hitl
