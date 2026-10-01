#pragma once

#include "learning/control_shadow.h"
#include "xlog_reader.h"

#include <cstdint>
#include <functional>
#include <string>

namespace hydrox::learning
{
  struct XLogControlFrame
  {
    uint64_t timestamp_ns = 0;
    uint64_t sequence = 0;
    ControlShadowInput input{};
    bool actuator_authorized = false;
    bool have_recorded_wrench = false;
    bool have_recorded_actuator = false;
    bool have_residual_policy = false;
    xlog::HydroxControllerOutputRecord recorded_wrench{};
    xlog::HydroxActuatorRecord recorded_actuator{};
    xlog::HydroxResidualPolicyRecord residual_policy{};
  };

  struct XLogControlOptions
  {
    bool require_timing = true;
    bool authorized_frames_only = true;
    xlog::ReaderOptions reader{};
  };

  struct XLogControlStats
  {
    uint64_t state_records = 0;
    uint64_t emitted_frames = 0;
    uint64_t skipped_without_setpoint = 0;
    uint64_t skipped_without_timing = 0;
    uint64_t skipped_unauthorized = 0;
    xlog::ReaderStats reader{};
  };

  using XLogControlCallback = std::function<bool(const XLogControlFrame &)>;

  // Assemble the sparse XLog topics into one explicit input per state tick.
  // State is the tick commit marker because the SITL writer emits it last.
  class XLogControlShadowReader final
  {
  public:
    bool read(const std::string & path,
              const XLogControlCallback & callback,
              std::string * error = nullptr,
              const XLogControlOptions & options = {});

    const XLogControlStats & stats() const { return stats_; }
    const std::string & metadata_json() const { return metadata_json_; }

  private:
    XLogControlStats stats_{};
    std::string metadata_json_;
  };
} // namespace hydrox::learning
