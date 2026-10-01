// Offline CSV control-stack replay.  This executable never opens a transport,
// DDS session, UE connection, or actuator device.
#include "control_parameters.h"
#include "hydrox/runtime/hil_session_config.h"
#include "learning/control_shadow.h"
#include "learning/xlog_control_shadow.h"
#include "vehicle_bundle.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
    constexpr const char *kInputHeader =
        "reset,dt_s,mode,eta_n,eta_e,eta_d,eta_roll,eta_pitch,eta_yaw,"
        "nu_u,nu_v,nu_w,nu_p,nu_q,nu_r,depth_m,sp_depth,sp_heading,sp_surge,"
        "sp_use_yaw_rate,sp_yaw_rate,sp_wp_n,sp_wp_e,sp_wp_d,action_x,action_m,"
        "action_n,confidence,valid";

    constexpr const char *kOutputHeader =
        "index,base_x,base_m,base_n,final_x,final_m,final_n,delta_x,delta_m,delta_n,"
        "base_ch0,base_ch1,base_ch2,base_ch3,base_ch4,base_ch5,base_ch6,base_ch7,base_rpm,"
        "final_ch0,final_ch1,final_ch2,final_ch3,final_ch4,final_ch5,final_ch6,final_ch7,final_rpm";

    constexpr const char *kXLogOutputHeader =
        "timestamp_ns,sequence,reset,mode,dt_s,"
        "recorded_x,recorded_y,recorded_z,recorded_k,recorded_m,recorded_n,"
        "shadow_x,shadow_y,shadow_z,shadow_k,shadow_m,shadow_n,"
        "tau_error_l2,tau_error_max,"
        "recorded_ch0,recorded_ch1,recorded_ch2,recorded_ch3,recorded_ch4,recorded_ch5,recorded_ch6,recorded_ch7,"
        "shadow_ch0,shadow_ch1,shadow_ch2,shadow_ch3,shadow_ch4,shadow_ch5,shadow_ch6,shadow_ch7,actuator_error_max";

    constexpr const char *kXLogStateColumns =
        "state_n,state_e,state_d,state_roll,state_pitch,state_yaw,"
        "state_u,state_v,state_w,state_p,state_q,state_r,state_depth,"
        "sp_depth,sp_heading,sp_surge,sp_use_yaw_rate,sp_yaw_rate,"
        "sp_wp_n,sp_wp_e,sp_wp_d,sp_use_path_segment,sp_path_start_n,"
        "sp_path_start_e,sp_lookahead,sp_arrival_radius,sp_hold_heading,"
        "actuator_authorized";

    constexpr const char *kXLogResidualPolicyColumns =
        "policy_audit_present,policy_mode,policy_observed,policy_candidate_valid,policy_active,"
        "policy_base_x,policy_base_y,policy_base_z,policy_base_k,policy_base_m,policy_base_n,"
        "policy_candidate_x,policy_candidate_y,policy_candidate_z,policy_candidate_k,policy_candidate_m,policy_candidate_n,"
        "policy_applied_x,policy_applied_y,policy_applied_z,policy_applied_k,policy_applied_m,policy_applied_n,"
        "policy_inference_ms,policy_pinn_ood,policy_selector_fraction_x,policy_selector_fraction_n,"
        "policy_requests,policy_accepted,policy_timeouts,policy_rejected_packets,policy_transport_errors";

    struct Options
    {
        std::string vehicle_bundle;
        std::string input_path;
        std::string xlog_path;
        std::string output_path;
        bool stdio = false;
        bool include_state = false;
        double blend = 0.10;
        double min_confidence = 0.75;
        std::array<double, 3> max_delta = {60.0, 8.0, 18.0};
        std::array<double, 3> max_rate = {120.0, 16.0, 36.0};
    };

    [[noreturn]] void usage(const char *message)
    {
        if (message && *message)
            std::fprintf(stderr, "hydrox_control_shadow: %s\n", message);
        std::fprintf(stderr,
            "Usage: hydrox_control_shadow [--stdio | --input FILE --output FILE | "
            "--xlog FILE --output FILE] "
            "--vehicle-bundle FILE "
            "[--blend 0.10] [--min-confidence 0.75] "
            "[--max-delta X,M,N] [--max-rate X,M,N] "
            "[--include-state]\n"
            "Input CSV header: %s\n",
            kInputHeader);
        std::exit(2);
    }

    std::vector<std::string> split_csv(const std::string &line)
    {
        std::vector<std::string> values;
        std::stringstream stream(line);
        std::string value;
        while (std::getline(stream, value, ','))
            values.push_back(value);
        return values;
    }

    void strip_trailing_carriage_return(std::string &line)
    {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
    }

    double parse_double(const std::string &value, const char *field)
    {
        std::size_t used = 0;
        const double result = std::stod(value, &used);
        if (used != value.size() || !std::isfinite(result))
            throw std::invalid_argument(std::string("invalid ") + field);
        return result;
    }

    int parse_flag(const std::string &value, const char *field)
    {
        const double parsed = parse_double(value, field);
        if (parsed != 0.0 && parsed != 1.0)
            throw std::invalid_argument(std::string(field) + " must be 0 or 1");
        return static_cast<int>(parsed);
    }

    std::array<double, 3> parse_triplet(const std::string &value, const char *field)
    {
        const auto parts = split_csv(value);
        if (parts.size() != 3)
            throw std::invalid_argument(std::string(field) + " must contain X,M,N");
        return {parse_double(parts[0], field), parse_double(parts[1], field),
                parse_double(parts[2], field)};
    }

    Options parse_options(int argc, char **argv)
    {
        Options options;
        for (int i = 1; i < argc; ++i)
        {
            const std::string option = argv[i];
            const auto require_value = [&](const char *name) -> const char *
            {
                if (++i >= argc)
                    usage(name);
                return argv[i];
            };
            if (option == "--vehicle-bundle") options.vehicle_bundle = require_value("--vehicle-bundle requires a value");
            else if (option == "--input") options.input_path = require_value("--input requires a value");
            else if (option == "--xlog") options.xlog_path = require_value("--xlog requires a value");
            else if (option == "--output") options.output_path = require_value("--output requires a value");
            else if (option == "--stdio") options.stdio = true;
            else if (option == "--include-state") options.include_state = true;
            else if (option == "--blend") options.blend = parse_double(require_value("--blend requires a value"), "blend");
            else if (option == "--min-confidence") options.min_confidence = parse_double(require_value("--min-confidence requires a value"), "min_confidence");
            else if (option == "--max-delta") options.max_delta = parse_triplet(require_value("--max-delta requires a value"), "max_delta");
            else if (option == "--max-rate") options.max_rate = parse_triplet(require_value("--max-rate requires a value"), "max_rate");
            else if (option == "--help" || option == "-h") usage("");
            else usage(("unknown option: " + option).c_str());
        }
        const bool has_input = !options.input_path.empty();
        const bool has_xlog = !options.xlog_path.empty();
        const bool has_output = !options.output_path.empty();
        const bool valid_stdio = options.stdio && !has_input && !has_xlog && !has_output;
        const bool valid_csv = !options.stdio && has_input && !has_xlog && has_output;
        const bool valid_xlog = !options.stdio && !has_input && has_xlog && has_output;
        if (!valid_stdio && !valid_csv && !valid_xlog)
            usage("select --stdio, --input FILE --output FILE, or --xlog FILE --output FILE");
        if (options.vehicle_bundle.empty()) usage("--vehicle-bundle is required");
        if (options.blend < 0.0 || options.blend > 1.0 || options.min_confidence < 0.0 ||
            options.min_confidence > 1.0)
            usage("blend and min-confidence must be in [0, 1]");
        for (double value : options.max_delta)
            if (value < 0.0) usage("max-delta must be non-negative");
        for (double value : options.max_rate)
            if (value < 0.0) usage("max-rate must be non-negative");
        return options;
    }

    std::string json_string_field(const std::string &json, const char *name)
    {
        const std::string prefix = "\"" + std::string(name) + "\":\"";
        const std::size_t begin = json.find(prefix);
        if (begin == std::string::npos)
            return {};
        const std::size_t value_begin = begin + prefix.size();
        const std::size_t value_end = json.find('"', value_begin);
        return value_end == std::string::npos
            ? std::string{}
            : json.substr(value_begin, value_end - value_begin);
    }

    bool json_true_field(const std::string &json, const char *name)
    {
        return json.find("\"" + std::string(name) + "\":true") !=
            std::string::npos;
    }

    std::string hex_u64(uint64_t value)
    {
        std::ostringstream stream;
        stream << std::hex << std::setw(16) << std::setfill('0') << value;
        return stream.str();
    }


    template<std::size_t Size>
    bool parse_hex_bytes(const std::string &value, std::array<uint8_t, Size> &bytes)
    {
        if (value.size() != Size * 2)
            return false;
        const auto nibble = [](char character) -> int
        {
            const unsigned char c = static_cast<unsigned char>(character);
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return c - 'a' + 10;
            if (c >= 'A' && c <= 'F') return c - 'A' + 10;
            return -1;
        };
        for (std::size_t index = 0; index < Size; ++index)
        {
            const int high = nibble(value[index * 2]);
            const int low = nibble(value[index * 2 + 1]);
            if (high < 0 || low < 0)
                return false;
            bytes[index] = static_cast<uint8_t>((high << 4) | low);
        }
        return true;
    }

    std::string hex_digest(const hydrox::runtime::HilSessionDigest &digest)
    {
        std::ostringstream stream;
        stream << std::hex << std::setfill('0');
        for (const uint8_t byte : digest)
            stream << std::setw(2) << static_cast<unsigned int>(byte);
        return stream.str();
    }

    std::string inspect_xlog_metadata(const std::string &path)
    {
        hydrox::xlog::Reader reader;
        std::string error;
        bool stopped_after_first_record = false;
        const bool ok = reader.read(
            path,
            [&](const hydrox::xlog::RecordView &)
            {
                stopped_after_first_record = true;
                return false;
            },
            &error);
        if (!ok && !stopped_after_first_record)
            throw std::runtime_error("cannot inspect XLog identity: " + error);
        return reader.metadata_json();
    }

    std::unordered_map<std::string, std::size_t> header_index(const std::string &header)
    {
        const auto expected = split_csv(kInputHeader);
        const auto actual = split_csv(header);
        if (actual != expected)
            throw std::invalid_argument("input CSV header does not match the documented control-shadow contract");
        std::unordered_map<std::string, std::size_t> index;
        for (std::size_t i = 0; i < actual.size(); ++i)
            index.emplace(actual[i], i);
        return index;
    }

    hydrox::learning::ControlShadowInput parse_input(
        const std::string &line, const std::unordered_map<std::string, std::size_t> &index)
    {
        const auto fields = split_csv(line);
        if (fields.size() != index.size())
            throw std::invalid_argument("row field count does not match input header");
        const auto value = [&](const char *name) -> const std::string & { return fields.at(index.at(name)); };
        hydrox::learning::ControlShadowInput input;
        input.reset_controller = parse_flag(value("reset"), "reset") != 0;
        input.dt_s = parse_double(value("dt_s"), "dt_s");
        const int mode = static_cast<int>(parse_double(value("mode"), "mode"));
        if (mode < static_cast<int>(hydrox::GNCMode::DISABLED) ||
            mode > static_cast<int>(hydrox::GNCMode::SURFACE))
            throw std::invalid_argument("mode is outside the HydroX GNCMode range");
        input.mode = static_cast<hydrox::GNCMode>(mode);
        const std::array<const char *, 6> eta_names = {"eta_n", "eta_e", "eta_d", "eta_roll", "eta_pitch", "eta_yaw"};
        const std::array<const char *, 6> nu_names = {"nu_u", "nu_v", "nu_w", "nu_p", "nu_q", "nu_r"};
        for (std::size_t i = 0; i < 6; ++i)
        {
            input.state.eta[static_cast<int>(i)] = parse_double(value(eta_names[i]), eta_names[i]);
            input.state.nu[static_cast<int>(i)] = parse_double(value(nu_names[i]), nu_names[i]);
        }
        input.state.depth_m = parse_double(value("depth_m"), "depth_m");
        input.setpoint.depth_ref = parse_double(value("sp_depth"), "sp_depth");
        input.setpoint.heading_ref = parse_double(value("sp_heading"), "sp_heading");
        input.setpoint.surge_ref = parse_double(value("sp_surge"), "sp_surge");
        input.setpoint.use_yaw_rate_ref = parse_flag(value("sp_use_yaw_rate"), "sp_use_yaw_rate") != 0;
        input.setpoint.yaw_rate_ref = parse_double(value("sp_yaw_rate"), "sp_yaw_rate");
        input.setpoint.wp_n = parse_double(value("sp_wp_n"), "sp_wp_n");
        input.setpoint.wp_e = parse_double(value("sp_wp_e"), "sp_wp_e");
        input.setpoint.wp_d = parse_double(value("sp_wp_d"), "sp_wp_d");
        input.residual.normalized.setZero();
        input.residual.normalized[0] = parse_double(value("action_x"), "action_x");
        input.residual.normalized[4] = parse_double(value("action_m"), "action_m");
        input.residual.normalized[5] = parse_double(value("action_n"), "action_n");
        input.residual.confidence = parse_double(value("confidence"), "confidence");
        input.residual.valid = parse_flag(value("valid"), "valid") != 0;
        return input;
    }

    void write_output(std::ostream &stream, std::size_t row,
                      const hydrox::learning::ControlShadowOutput &output)
    {
        stream << std::setprecision(17) << row << ','
               << output.base_wrench[0] << ',' << output.base_wrench[4] << ',' << output.base_wrench[5] << ','
               << output.final_wrench[0] << ',' << output.final_wrench[4] << ',' << output.final_wrench[5] << ','
               << output.applied_delta[0] << ',' << output.applied_delta[4] << ',' << output.applied_delta[5];
        for (float value : output.base_actuator.ch)
            stream << ',' << value;
        stream << ',' << output.base_actuator.rpm;
        for (float value : output.final_actuator.ch)
            stream << ',' << value;
        stream << ',' << output.final_actuator.rpm << '\n';
    }

    void write_xlog_output(
        std::ostream &stream,
        const hydrox::learning::XLogControlFrame &frame,
        const hydrox::learning::ControlShadowOutput &output,
        bool include_state)
    {
        const double missing = std::numeric_limits<double>::quiet_NaN();
        hydrox::Wrench recorded = hydrox::Wrench::Constant(missing);
        if (frame.have_recorded_wrench)
            for (int axis = 0; axis < 6; ++axis)
                recorded[axis] = frame.recorded_wrench.tau[axis];

        double tau_l2 = missing;
        double tau_max = missing;
        if (frame.have_recorded_wrench)
        {
            const hydrox::Wrench difference = output.base_wrench - recorded;
            tau_l2 = difference.norm();
            tau_max = difference.cwiseAbs().maxCoeff();
        }

        stream << std::setprecision(17)
               << frame.timestamp_ns << ',' << frame.sequence << ','
               << (frame.input.reset_controller ? 1 : 0) << ','
               << static_cast<int>(frame.input.mode) << ',' << frame.input.dt_s;
        if (include_state)
        {
            for (int axis = 0; axis < 6; ++axis)
                stream << ',' << frame.input.state.eta[axis];
            for (int axis = 0; axis < 6; ++axis)
                stream << ',' << frame.input.state.nu[axis];
            const auto &setpoint = frame.input.setpoint;
            stream << ',' << frame.input.state.depth_m
                   << ',' << setpoint.depth_ref
                   << ',' << setpoint.heading_ref
                   << ',' << setpoint.surge_ref
                   << ',' << (setpoint.use_yaw_rate_ref ? 1 : 0)
                   << ',' << setpoint.yaw_rate_ref
                   << ',' << setpoint.wp_n
                   << ',' << setpoint.wp_e
                   << ',' << setpoint.wp_d
                   << ',' << (setpoint.use_path_segment ? 1 : 0)
                   << ',' << setpoint.path_start_n
                   << ',' << setpoint.path_start_e
                   << ',' << setpoint.lookahead_m
                   << ',' << setpoint.arrival_radius_m
                   << ',' << (setpoint.hold_heading ? 1 : 0)
                   << ',' << (frame.actuator_authorized ? 1 : 0)
                   << ',' << (frame.have_residual_policy ? 1 : 0);
            const auto policy_value = [&](double value)
            {
                stream << ',' << (frame.have_residual_policy ? value : missing);
            };
            policy_value(static_cast<double>(frame.residual_policy.mode));
            policy_value(static_cast<double>(frame.residual_policy.observed));
            policy_value(static_cast<double>(frame.residual_policy.candidate_valid));
            policy_value(static_cast<double>(frame.residual_policy.active));
            for (double value : frame.residual_policy.base_tau)
                policy_value(value);
            for (double value : frame.residual_policy.candidate_tau)
                policy_value(value);
            for (double value : frame.residual_policy.applied_tau)
                policy_value(value);
            policy_value(frame.residual_policy.inference_ms);
            policy_value(frame.residual_policy.pinn_ood_feature);
            policy_value(frame.residual_policy.selector_fraction_x_n[0]);
            policy_value(frame.residual_policy.selector_fraction_x_n[1]);
            policy_value(static_cast<double>(frame.residual_policy.requests));
            policy_value(static_cast<double>(frame.residual_policy.accepted));
            policy_value(static_cast<double>(frame.residual_policy.timeouts));
            policy_value(static_cast<double>(frame.residual_policy.rejected_packets));
            policy_value(static_cast<double>(frame.residual_policy.transport_errors));
        }
        for (int axis = 0; axis < 6; ++axis)
            stream << ',' << recorded[axis];
        for (int axis = 0; axis < 6; ++axis)
            stream << ',' << output.base_wrench[axis];
        stream << ',' << tau_l2 << ',' << tau_max;

        double actuator_error_max = missing;
        if (frame.have_recorded_actuator)
        {
            actuator_error_max = 0.0;
            for (std::size_t channel = 0; channel < output.base_actuator.ch.size(); ++channel)
                actuator_error_max = std::max(
                    actuator_error_max,
                    std::abs(static_cast<double>(output.base_actuator.ch[channel]) -
                             static_cast<double>(frame.recorded_actuator.ch[channel])));
        }
        for (std::size_t channel = 0; channel < output.base_actuator.ch.size(); ++channel)
            stream << ',' << (frame.have_recorded_actuator
                ? static_cast<double>(frame.recorded_actuator.ch[channel]) : missing);
        for (float value : output.base_actuator.ch)
            stream << ',' << value;
        stream << ',' << actuator_error_max << '\n';
    }

    int run(const Options &options)
    {
        std::string params_error;
        const auto bundle = hydrox::load_vehicle_bundle(options.vehicle_bundle, &params_error);
        const auto& params = bundle.control;
        if (!bundle.valid)
            throw std::runtime_error("unable to load vehicle parameters: " + params_error);

        if (!options.xlog_path.empty())
        {
            const std::string metadata = inspect_xlog_metadata(options.xlog_path);
            if (json_true_field(metadata, "profile_bound"))
            {
                const std::string selected_profile_id = bundle.id;
                const uint64_t selected_profile_fingerprint = bundle.fingerprint;
                const std::string expected_id = json_string_field(metadata, "profile_id");
                const std::string expected_fingerprint =
                    json_string_field(metadata, "profile_fingerprint");
                const std::string session_digest =
                    json_string_field(metadata, "session_digest");
                const std::string session_payload_hex =
                    json_string_field(metadata, "session_config_payload");
                if (expected_id.empty() || expected_id != selected_profile_id ||
                    expected_fingerprint != hex_u64(selected_profile_fingerprint))
                    throw std::runtime_error(
                        "VehicleBundle ID/fingerprint does not match the XLog Profile identity");
                hydrox::runtime::HilSessionConfigPayload session_payload{};
                hydrox::runtime::HilSessionConfigV1 session_config;
                hydrox::runtime::HilSessionField invalid_field =
                    hydrox::runtime::HilSessionField::None;
                if (!parse_hex_bytes(session_payload_hex, session_payload) ||
                    !hydrox::runtime::decode_hil_session_config(
                        session_payload.data(), session_payload.size(), session_config) ||
                    !hydrox::runtime::validate_hil_session_config(
                        session_config, invalid_field))
                    throw std::runtime_error(
                        "profile-bound XLog has a missing or invalid canonical Session config");
                const hydrox::runtime::HilSessionDigest computed_digest =
                    hydrox::runtime::hil_session_digest(
                        selected_profile_fingerprint, session_config);
                if (session_digest != hex_digest(computed_digest))
                    throw std::runtime_error(
                        "Session digest does not match the recorded Profile and Session config");
            }
            else
            {
                std::fprintf(
                    stderr,
                    "hydrox_control_shadow: WARNING: XLog is not Profile-bound; "
                    "the replay is diagnostic, not configuration-qualified\n");
            }
        }

        hydrox::learning::ResidualSafetyFilter::Params safety;
        safety.enabled = true;
        safety.blend = options.blend;
        safety.min_confidence = options.min_confidence;
        safety.max_delta[0] = options.max_delta[0];
        safety.max_delta[4] = options.max_delta[1];
        safety.max_delta[5] = options.max_delta[2];
        safety.max_rate[0] = options.max_rate[0];
        safety.max_rate[4] = options.max_rate[1];
        safety.max_rate[5] = options.max_rate[2];
        hydrox::learning::ControlShadowReplay replay(params, safety);

        if (!options.xlog_path.empty())
        {
            std::ofstream output(options.output_path);
            if (!output.is_open())
                throw std::runtime_error("cannot open XLog replay output CSV");
            if (options.include_state)
            {
                const std::string header = kXLogOutputHeader;
                const std::size_t insert_at = header.find("recorded_x");
                output << header.substr(0, insert_at) << kXLogStateColumns << ','
                       << kXLogResidualPolicyColumns << ','
                       << header.substr(insert_at) << '\n';
            }
            else
            {
                output << kXLogOutputHeader << '\n';
            }
            hydrox::learning::XLogControlShadowReader input;
            hydrox::learning::XLogControlOptions xlog_options;
            std::string read_error;
            if (!input.read(
                options.xlog_path,
                [&](const hydrox::learning::XLogControlFrame &frame)
                {
                    write_xlog_output(
                        output, frame, replay.step(frame.input), options.include_state);
                    return output.good();
                },
                &read_error,
                xlog_options))
            {
                throw std::runtime_error("XLog replay failed: " + read_error);
            }
            const auto &stats = input.stats();
            std::fprintf(
                stderr,
                "hydrox_control_shadow: XLog frames=%llu states=%llu skipped_no_setpoint=%llu "
                "skipped_no_timing=%llu skipped_unauthorized=%llu corrupt_blocks=%llu sequence_gaps=%llu\n",
                static_cast<unsigned long long>(stats.emitted_frames),
                static_cast<unsigned long long>(stats.state_records),
                static_cast<unsigned long long>(stats.skipped_without_setpoint),
                static_cast<unsigned long long>(stats.skipped_without_timing),
                static_cast<unsigned long long>(stats.skipped_unauthorized),
                static_cast<unsigned long long>(stats.reader.corrupt_block_count),
                static_cast<unsigned long long>(stats.reader.sequence_gap_count));
            return 0;
        }

        std::unique_ptr<std::ifstream> input_file;
        std::unique_ptr<std::ofstream> output_file;
        std::istream *input = &std::cin;
        std::ostream *output = &std::cout;
        if (!options.stdio)
        {
            input_file = std::make_unique<std::ifstream>(options.input_path);
            output_file = std::make_unique<std::ofstream>(options.output_path);
            if (!input_file->is_open()) throw std::runtime_error("cannot open input CSV");
            if (!output_file->is_open()) throw std::runtime_error("cannot open output CSV");
            input = input_file.get();
            output = output_file.get();
        }

        std::string header;
        if (!std::getline(*input, header))
            throw std::runtime_error("input CSV is empty");
        strip_trailing_carriage_return(header);
        const auto index = header_index(header);
        *output << kOutputHeader << '\n';
        output->flush();
        std::string line;
        std::size_t row = 0;
        while (std::getline(*input, line))
        {
            strip_trailing_carriage_return(line);
            if (line.empty()) continue;
            const auto shadow_input = parse_input(line, index);
            write_output(*output, row++, replay.step(shadow_input));
            // The persistent Python client needs one response per row.  Batch
            // file replay has no such latency requirement and should let the
            // stream buffer efficiently for large XLog-derived datasets.
            if (options.stdio)
                output->flush();
        }
        return 0;
    }
} // namespace

int main(int argc, char **argv)
{
    try
    {
        return run(parse_options(argc, argv));
    }
    catch (const std::exception &error)
    {
        std::fprintf(stderr, "hydrox_control_shadow: %s\n", error.what());
        return 1;
    }
}
