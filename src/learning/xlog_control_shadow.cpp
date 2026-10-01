#include "learning/xlog_control_shadow.h"

#include <cmath>
#include <cstring>

namespace hydrox::learning
{
namespace
{
  template<typename T>
  bool decode_exact(const xlog::RecordView & view, T & output)
  {
    if (view.header.payload_size != sizeof(T) || view.payload == nullptr)
      return false;
    std::memcpy(&output, view.payload, sizeof(T));
    return true;
  }

  void apply_setpoint(const xlog::HydroxSetpointRecord & record,
                      GNCSetpoint & setpoint)
  {
    setpoint = {};
    setpoint.depth_ref = record.depth_ref;
    setpoint.heading_ref = record.heading_ref;
    setpoint.surge_ref = record.surge_ref;
    setpoint.yaw_rate_ref = record.yaw_rate_ref;
    setpoint.wp_n = record.wp_n;
    setpoint.wp_e = record.wp_e;
    setpoint.wp_d = record.wp_d;
    setpoint.use_yaw_rate_ref = record.use_yaw_rate_ref != 0;
    setpoint.use_path_segment = record.use_path_segment != 0;
    setpoint.hold_heading = record.hold_heading != 0;
    setpoint.path_start_n = record.path_start_n;
    setpoint.path_start_e = record.path_start_e;
    setpoint.lookahead_m = record.lookahead_m;
    setpoint.arrival_radius_m = record.arrival_radius_m;
  }

  bool valid_mode(uint8_t raw)
  {
    return raw <= static_cast<uint8_t>(GNCMode::SURFACE);
  }
} // namespace

bool XLogControlShadowReader::read(const std::string & path,
                                   const XLogControlCallback & callback,
                                   std::string * error,
                                   const XLogControlOptions & options)
{
  stats_ = {};
  metadata_json_.clear();

  GNCSetpoint setpoint{};
  bool have_setpoint = false;
  xlog::HydroxTimingRecord timing{};
  uint64_t timing_timestamp = 0;
  bool have_timing = false;
  xlog::HydroxControllerOutputRecord wrench{};
  uint64_t wrench_timestamp = 0;
  bool have_wrench = false;
  xlog::HydroxActuatorRecord actuator{};
  uint64_t actuator_timestamp = 0;
  bool have_actuator = false;
  xlog::HydroxResidualPolicyRecord residual_policy{};
  uint64_t residual_policy_timestamp = 0;
  bool have_residual_policy = false;
  bool first_frame = true;
  bool pending_reset = true;
  GNCMode previous_mode = GNCMode::DISABLED;
  std::string adapter_error;

  xlog::Reader reader;
  const bool read_ok = reader.read(
    path,
    [&](const xlog::RecordView & view)
    {
      const auto topic = static_cast<xlog::TopicId>(view.header.topic_id);
      if (topic == xlog::TopicId::HydroxSetpoint)
      {
        xlog::HydroxSetpointRecord record{};
        if (!decode_exact(view, record))
        {
          adapter_error = "HydroxSetpoint requires a complete schema v2 record";
          return false;
        }
        apply_setpoint(record, setpoint);
        have_setpoint = true;
        return true;
      }

      if (topic == xlog::TopicId::HydroxTiming)
      {
        if (!decode_exact(view, timing))
        {
          adapter_error = "invalid HydroxTiming payload size";
          return false;
        }
        timing_timestamp = view.header.timestamp_ns;
        have_timing = true;
        return true;
      }

      if (topic == xlog::TopicId::HydroxControllerOutput)
      {
        if (!decode_exact(view, wrench))
        {
          adapter_error = "invalid HydroxControllerOutput payload size";
          return false;
        }
        wrench_timestamp = view.header.timestamp_ns;
        have_wrench = true;
        return true;
      }

      if (topic == xlog::TopicId::HydroxActuator)
      {
        if (!decode_exact(view, actuator))
        {
          adapter_error = "invalid HydroxActuator payload size";
          return false;
        }
        actuator_timestamp = view.header.timestamp_ns;
        have_actuator = true;
        return true;
      }

      if (topic == xlog::TopicId::HydroxResidualPolicy)
      {
        if (!decode_exact(view, residual_policy))
        {
          adapter_error = "invalid HydroxResidualPolicy payload size";
          return false;
        }
        residual_policy_timestamp = view.header.timestamp_ns;
        have_residual_policy = true;
        return true;
      }

      if (topic != xlog::TopicId::HydroxState)
        return true;

      ++stats_.state_records;
      xlog::HydroxStateRecord state{};
      if (!decode_exact(view, state))
      {
        adapter_error = "invalid HydroxState payload size";
        return false;
      }
      if (!valid_mode(state.gnc_mode))
      {
        adapter_error = "HydroxState contains an invalid GNC mode";
        return false;
      }
      pending_reset = pending_reset || state.controller_reset != 0 ||
        static_cast<GNCMode>(state.gnc_mode) != previous_mode;
      previous_mode = static_cast<GNCMode>(state.gnc_mode);
      if (options.authorized_frames_only && state.actuator_authorized == 0)
      {
        ++stats_.skipped_unauthorized;
        return true;
      }
      if (!have_setpoint)
      {
        ++stats_.skipped_without_setpoint;
        return true;
      }
      const bool matching_timing =
        have_timing && timing_timestamp == view.header.timestamp_ns &&
        std::isfinite(timing.dt) && timing.dt > 0.0;
      if (!matching_timing && options.require_timing)
      {
        ++stats_.skipped_without_timing;
        return true;
      }

      XLogControlFrame frame;
      frame.timestamp_ns = view.header.timestamp_ns;
      frame.sequence = view.header.sequence;
      frame.actuator_authorized = state.actuator_authorized != 0;
      frame.input.state = NavigationState::zeros();
      for (int index = 0; index < 6; ++index)
      {
        frame.input.state.eta[index] = state.eta[index];
        frame.input.state.nu[index] = state.nu[index];
      }
      frame.input.state.depth_m = state.depth_m;
      frame.input.state.dvl_valid = state.dvl_valid != 0;
      frame.input.state.timestamp = static_cast<double>(view.header.timestamp_ns) * 1.0e-9;
      frame.input.setpoint = setpoint;
      frame.input.mode = static_cast<GNCMode>(state.gnc_mode);
      frame.input.dt_s = matching_timing ? timing.dt : 0.01;
      frame.input.reset_controller = first_frame || pending_reset;
      // XLog currently records the final applied wrench, not a policy action.
      // Replaying with an invented residual would be misleading, so the
      // adapter explicitly requests baseline control only.
      frame.input.residual = {};
      frame.have_recorded_wrench = have_wrench &&
        wrench_timestamp == view.header.timestamp_ns;
      frame.have_recorded_actuator = have_actuator &&
        actuator_timestamp == view.header.timestamp_ns;
      frame.have_residual_policy = have_residual_policy &&
        residual_policy_timestamp == view.header.timestamp_ns;
      if (frame.have_recorded_wrench)
        frame.recorded_wrench = wrench;
      if (frame.have_recorded_actuator)
        frame.recorded_actuator = actuator;
      if (frame.have_residual_policy)
        frame.residual_policy = residual_policy;

      if (callback && !callback(frame))
      {
        adapter_error = "control-shadow XLog callback aborted";
        return false;
      }
      first_frame = false;
      pending_reset = false;
      ++stats_.emitted_frames;
      return true;
    },
    error,
    options.reader);

  metadata_json_ = reader.metadata_json();
  stats_.reader = reader.stats();
  if (!read_ok && !adapter_error.empty() && error)
    *error = adapter_error;
  if (!read_ok)
    return false;
  if (stats_.emitted_frames == 0)
  {
    if (error)
      *error = "XLog contains no complete control frames";
    return false;
  }
  return true;
}
} // namespace hydrox::learning
