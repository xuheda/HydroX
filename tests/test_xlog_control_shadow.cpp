#include "vehicle_bundle.h"
#include "hydrox/runtime/hil_session_config.h"
#include "learning/control_shadow.h"
#include "learning/xlog_control_shadow.h"
#include "vehicle_bundle.h"
#include "xlog_writer.h"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#ifdef _WIN32
#include <process.h>
#endif

namespace
{
  int fail(const char * message)
  {
    std::fprintf(stderr, "FAIL: %s\n", message);
    return 1;
  }

  template<typename Bytes>
  std::string bytes_hex(const Bytes & bytes)
  {
    static constexpr char digits[] = "0123456789abcdef";
    std::string result;
    result.reserve(bytes.size() * 2);
    for (const uint8_t byte : bytes)
    {
      result.push_back(digits[byte >> 4]);
      result.push_back(digits[byte & 0x0f]);
    }
    return result;
  }

  std::string hex_u64(uint64_t value)
  {
    static constexpr char digits[] = "0123456789abcdef";
    std::string result(16, '0');
    for (int index = 15; index >= 0; --index)
    {
      result[static_cast<std::size_t>(index)] = digits[value & 0x0f];
      value >>= 4;
    }
    return result;
  }

  std::string bound_metadata(
      const std::string & profile_id,
      uint64_t profile_fingerprint,
      const hydrox::runtime::HilSessionConfigV1 & session,
      const hydrox::runtime::HilSessionDigest & digest)
  {
    return "{\"profile_bound\":true,\"profile_id\":\"" + profile_id +
      "\",\"profile_fingerprint\":\"" + hex_u64(profile_fingerprint) +
      "\",\"session_config_payload\":\"" +
      bytes_hex(hydrox::runtime::encode_hil_session_config(session)) +
      "\",\"session_digest\":\"" + bytes_hex(digest) + "\"}";
  }


  hydrox::xlog::HydroxSetpointRecord make_setpoint_record(
      const hydrox::GNCSetpoint & setpoint)
  {
    hydrox::xlog::HydroxSetpointRecord record;
    record.depth_ref = setpoint.depth_ref;
    record.heading_ref = setpoint.heading_ref;
    record.surge_ref = setpoint.surge_ref;
    record.yaw_rate_ref = setpoint.yaw_rate_ref;
    record.wp_n = setpoint.wp_n;
    record.wp_e = setpoint.wp_e;
    record.wp_d = setpoint.wp_d;
    record.use_yaw_rate_ref = setpoint.use_yaw_rate_ref ? 1u : 0u;
    record.use_path_segment = setpoint.use_path_segment ? 1u : 0u;
    record.hold_heading = setpoint.hold_heading ? 1u : 0u;
    record.path_start_n = setpoint.path_start_n;
    record.path_start_e = setpoint.path_start_e;
    record.lookahead_m = setpoint.lookahead_m;
    record.arrival_radius_m = setpoint.arrival_radius_m;
    return record;
  }

  bool write_tick(hydrox::xlog::Writer & writer,
                  uint64_t timestamp_ns,
                  const hydrox::learning::ControlShadowInput & input,
                  const hydrox::learning::ControlShadowOutput & output)
  {
    hydrox::xlog::HydroxTimingRecord timing;
    timing.dt = input.dt_s;
    timing.expected_dt = input.dt_s;
    if (!writer.write(hydrox::xlog::TopicId::HydroxTiming,
                      timestamp_ns, timing))
      return false;

    hydrox::xlog::HydroxControllerOutputRecord wrench;
    for (int axis = 0; axis < 6; ++axis)
      wrench.tau[axis] = output.base_wrench[axis];
    wrench.tau_norm = output.base_wrench.norm();
    if (!writer.write(hydrox::xlog::TopicId::HydroxControllerOutput,
                      timestamp_ns, wrench))
      return false;

    hydrox::xlog::HydroxActuatorRecord actuator;
    for (std::size_t channel = 0; channel < output.base_actuator.ch.size(); ++channel)
      actuator.ch[channel] = output.base_actuator.ch[channel];
    actuator.rpm = output.base_actuator.rpm;
    actuator.active_count = output.base_actuator.active_count;
    if (!writer.write(hydrox::xlog::TopicId::HydroxActuator,
                      timestamp_ns, actuator))
      return false;

    hydrox::xlog::HydroxResidualPolicyRecord residual_policy;
    for (int axis = 0; axis < 6; ++axis)
    {
      residual_policy.base_tau[axis] = output.base_wrench[axis];
      residual_policy.candidate_tau[axis] = output.base_wrench[axis];
      residual_policy.applied_tau[axis] = output.base_wrench[axis];
    }
    residual_policy.inference_ms = 2.5;
    residual_policy.pinn_ood_feature = 0.01;
    residual_policy.selector_fraction_x_n[0] = 0.75;
    residual_policy.selector_fraction_x_n[1] = 0.50;
    residual_policy.requests = 1;
    residual_policy.accepted = 1;
    residual_policy.observed = 1;
    residual_policy.candidate_valid = 1;
    residual_policy.mode = 1;
    if (!writer.write(hydrox::xlog::TopicId::HydroxResidualPolicy,
                      timestamp_ns, residual_policy))
      return false;

    hydrox::xlog::HydroxStateRecord state;
    for (int axis = 0; axis < 6; ++axis)
    {
      state.eta[axis] = input.state.eta[axis];
      state.nu[axis] = input.state.nu[axis];
    }
    state.depth_m = input.state.depth_m;
    state.gnc_mode = static_cast<uint8_t>(input.mode);
    state.controller_reset = input.reset_controller ? 1u : 0u;
    state.actuator_authorized = 1u;
    return writer.write(
      hydrox::xlog::TopicId::HydroxState, timestamp_ns, state);
  }
} // namespace

int main(int argc, char ** argv)
{
  using namespace hydrox;
  using namespace hydrox::learning;
  using namespace hydrox::xlog;

  VehicleBundle bundle;
  ControlParameters params;
  std::string error;
  if (argc > 2)
  {
    bundle = load_vehicle_bundle(argv[2], &error);
    params = bundle.control;
  }
  else
  {
    params = load_vehicle_bundle("profiles/eca-a9/vehicle-bundle.json").control;
  }
  if (!params.valid)
    return fail("control parameters");
  ResidualSafetyFilter::Params safety;

  const std::filesystem::path directory =
    std::filesystem::temp_directory_path() /
    ("hydrox_test_xlog_shadow_" + std::to_string(unix_time_ns_now()));
  std::filesystem::create_directories(directory);
  const std::filesystem::path path = directory / "shadow.xlog";

  ControlShadowReplay source_replay(params, safety);
  ControlShadowInput first;
  first.reset_controller = true;
  first.dt_s = 0.01;
  first.mode = GNCMode::WAYPOINT_3D;
  first.state = NavigationState::zeros();
  first.state.eta[0] = 3.0;
  first.state.eta[1] = -2.0;
  first.state.eta[2] = 4.0;
  first.state.depth_m = 4.0;
  first.state.nu[0] = 0.4;
  first.setpoint.depth_ref = 7.0;
  first.setpoint.heading_ref = 0.25;
  first.setpoint.surge_ref = 1.2;
  first.setpoint.wp_n = 40.0;
  first.setpoint.wp_e = 15.0;
  first.setpoint.wp_d = 7.0;
  first.setpoint.use_path_segment = true;
  first.setpoint.path_start_n = -10.0;
  first.setpoint.path_start_e = -5.0;
  first.setpoint.lookahead_m = 9.0;
  first.setpoint.arrival_radius_m = 2.5;

  ControlShadowInput second = first;
  second.reset_controller = false;
  second.state.eta[0] += 0.01;
  second.state.eta[2] += 0.002;
  second.state.depth_m = second.state.eta[2];
  second.state.nu[0] += 0.02;

  const std::vector<ControlShadowInput> source_inputs = {first, second};
  std::vector<ControlShadowOutput> expected;
  Writer writer;
  runtime::HilSessionConfigV1 session;
  const runtime::HilSessionDigest session_digest =
    runtime::hil_session_digest(bundle.fingerprint, session);
  const std::string metadata = bundle.valid
    ? bound_metadata(bundle.id, bundle.fingerprint, session, session_digest)
    : R"({"vehicle":"shadow_test"})";
  if (!writer.open(path.string(), metadata, &error))
    return fail(error.c_str());
  const auto setpoint_record = make_setpoint_record(first.setpoint);
  if (!writer.write(TopicId::HydroxSetpoint, 1'000'000'000ULL, setpoint_record))
    return fail("write v2 setpoint");
  for (std::size_t index = 0; index < source_inputs.size(); ++index)
  {
    expected.push_back(source_replay.step(source_inputs[index]));
    if (!write_tick(writer, 1'000'000'000ULL + index * 10'000'000ULL,
                    source_inputs[index], expected.back()))
      return fail("write control tick");
  }
  writer.close();

  XLogControlShadowReader adapter;
  ControlShadowReplay replay(params, safety);
  std::size_t frame_index = 0;
  if (!adapter.read(
      path.string(),
      [&](const XLogControlFrame & frame)
      {
        if (frame_index >= expected.size() ||
            !frame.actuator_authorized || !frame.have_recorded_wrench ||
            !frame.have_recorded_actuator || !frame.have_residual_policy)
          return false;
        if (frame.residual_policy.inference_ms != 2.5 ||
            frame.residual_policy.accepted != 1 ||
            frame.residual_policy.mode != 1)
          return false;
        if (frame.input.setpoint.use_path_segment != first.setpoint.use_path_segment ||
            frame.input.setpoint.path_start_n != first.setpoint.path_start_n ||
            frame.input.setpoint.path_start_e != first.setpoint.path_start_e ||
            frame.input.setpoint.lookahead_m != first.setpoint.lookahead_m ||
            frame.input.setpoint.arrival_radius_m != first.setpoint.arrival_radius_m ||
            frame.input.reset_controller != (frame_index == 0))
          return false;

        const ControlShadowOutput actual = replay.step(frame.input);
        if ((actual.base_wrench - expected[frame_index].base_wrench).norm() > 1.0e-10)
          return false;
        for (std::size_t channel = 0; channel < actual.base_actuator.ch.size(); ++channel)
          if (std::abs(actual.base_actuator.ch[channel] -
                       expected[frame_index].base_actuator.ch[channel]) > 1.0e-6f)
            return false;
        ++frame_index;
        return true;
      },
      &error))
    return fail(error.c_str());
  if (frame_index != expected.size() || adapter.stats().emitted_frames != 2)
    return fail("v2 XLog control replay frame count");

  const std::filesystem::path legacy_path = directory / "legacy.xlog";
  Writer legacy_writer;
  if (!legacy_writer.open(legacy_path.string(), "{}", &error))
    return fail("open legacy fixture");
  HydroxSetpointRecordV1 legacy_setpoint;
  legacy_setpoint.depth_ref = first.setpoint.depth_ref;
  legacy_setpoint.heading_ref = first.setpoint.heading_ref;
  legacy_setpoint.surge_ref = first.setpoint.surge_ref;
  if (!legacy_writer.write(TopicId::HydroxSetpoint, 2'000'000'000ULL,
                           legacy_setpoint) ||
      !write_tick(legacy_writer, 2'000'000'000ULL, first, expected.front()))
    return fail("write legacy fixture");
  legacy_writer.close();

  XLogControlShadowReader reject_legacy;
  if (reject_legacy.read(legacy_path.string(), {}, &error))
    return fail("default replay accepted incomplete v1 setpoint");
  if (reject_legacy.stats().emitted_frames != 0 ||
      error.find("complete schema v2") == std::string::npos)
    return fail("incomplete v1 setpoint must fail before emitting frames");

  if (argc > 1)
  {
    const std::filesystem::path output_path = directory / "shadow_report.csv";
    const auto run_cli = [&](const std::filesystem::path & input_path,
                             const char * bundle_path) -> int
    {
#ifdef _WIN32
      const std::string xlog_argument = input_path.string();
      const std::string output_argument = output_path.string();
      if (bundle_path != nullptr)
      {
        const char * arguments[] = {
          argv[1], "--xlog", xlog_argument.c_str(), "--output",
          output_argument.c_str(), "--vehicle-bundle", bundle_path,
          "--include-state", nullptr};
        return static_cast<int>(_spawnv(_P_WAIT, argv[1], arguments));
      }
      const char * arguments[] = {
        argv[1], "--xlog", xlog_argument.c_str(), "--output",
        output_argument.c_str(), "--vehicle-bundle", "profiles/eca-a9/vehicle-bundle.json",
        "--include-state", nullptr};
      return static_cast<int>(_spawnv(_P_WAIT, argv[1], arguments));
#else
      std::string command =
        "\"" + std::string(argv[1]) + "\" --xlog \"" + input_path.string() +
        "\" --output \"" + output_path.string() + "\" --include-state";
      command += bundle_path != nullptr
        ? " --vehicle-bundle \"" + std::string(bundle_path) + "\""
        : " --vehicle-bundle profiles/eca-a9/vehicle-bundle.json";
      return std::system(command.c_str());
#endif
    };
    const int command_status = run_cli(path, argc > 2 ? argv[2] : nullptr);
    if (command_status != 0)
      return fail("hydrox_control_shadow --xlog CLI");
    std::ifstream report(output_path);
    std::string line;
    std::size_t lines = 0;
    while (std::getline(report, line))
    {
      if (lines == 0 &&
          (line.find("tau_error_l2") == std::string::npos ||
           line.find("policy_audit_present") == std::string::npos ||
           line.find("policy_transport_errors") == std::string::npos))
        return fail("XLog shadow report audit header");
      ++lines;
    }
    if (lines != 3)
      return fail("XLog shadow report row count");

    if (argc > 3 && run_cli(path, argv[3]) == 0)
      return fail("CLI accepted the wrong VehicleBundle");

    if (bundle.valid)
    {
      const std::filesystem::path bad_session_path =
        directory / "bad_session.xlog";
      runtime::HilSessionConfigV1 changed_session = session;
      changed_session.mission_radius_mm += 1;
      Writer bad_session_writer;
      if (!bad_session_writer.open(
            bad_session_path.string(),
            bound_metadata(bundle.id, bundle.fingerprint,
                           changed_session, session_digest), &error))
        return fail("open invalid Session identity fixture");
      bad_session_writer.close();
      if (run_cli(bad_session_path, argv[2]) == 0)
        return fail("CLI accepted a mismatched Session digest");
    }


  }

  std::error_code cleanup_error;
  std::filesystem::remove_all(directory, cleanup_error);
  std::printf("test_xlog_control_shadow: all checks passed\n");
  return 0;
}
