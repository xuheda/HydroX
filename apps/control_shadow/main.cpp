// Offline CSV control-stack replay.  This executable never opens a transport,
// DDS session, UE connection, or actuator device.
#include "fossen_vehicle_params.h"
#include "learning/control_shadow.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
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

    struct Options
    {
        std::string vehicle = "EcaA9";
        std::string vehicle_params;
        std::string vehicle_params_dir;
        std::string input_path;
        std::string output_path;
        bool stdio = false;
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
            "Usage: hydrox_control_shadow [--stdio | --input FILE --output FILE] "
            "[--vehicle EcaA9] [--vehicle-params FILE] [--vehicle-params-dir DIR] "
            "[--blend 0.10] [--min-confidence 0.75] "
            "[--max-delta X,M,N] [--max-rate X,M,N]\n"
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
            if (option == "--vehicle") options.vehicle = require_value("--vehicle requires a value");
            else if (option == "--vehicle-params") options.vehicle_params = require_value("--vehicle-params requires a value");
            else if (option == "--vehicle-params-dir") options.vehicle_params_dir = require_value("--vehicle-params-dir requires a value");
            else if (option == "--input") options.input_path = require_value("--input requires a value");
            else if (option == "--output") options.output_path = require_value("--output requires a value");
            else if (option == "--stdio") options.stdio = true;
            else if (option == "--blend") options.blend = parse_double(require_value("--blend requires a value"), "blend");
            else if (option == "--min-confidence") options.min_confidence = parse_double(require_value("--min-confidence requires a value"), "min_confidence");
            else if (option == "--max-delta") options.max_delta = parse_triplet(require_value("--max-delta requires a value"), "max_delta");
            else if (option == "--max-rate") options.max_rate = parse_triplet(require_value("--max-rate requires a value"), "max_rate");
            else if (option == "--help" || option == "-h") usage("");
            else usage(("unknown option: " + option).c_str());
        }
        const bool has_input = !options.input_path.empty();
        const bool has_output = !options.output_path.empty();
        if (options.stdio ? (has_input || has_output) : (!has_input || !has_output))
            usage("select either --stdio or both --input and --output");
        if (options.blend < 0.0 || options.blend > 1.0 || options.min_confidence < 0.0 ||
            options.min_confidence > 1.0)
            usage("blend and min-confidence must be in [0, 1]");
        for (double value : options.max_delta)
            if (value < 0.0) usage("max-delta must be non-negative");
        for (double value : options.max_rate)
            if (value < 0.0) usage("max-rate must be non-negative");
        return options;
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

    int run(const Options &options)
    {
        std::string params_error;
        const auto params = hydrox::load_fossen_control_params(
            options.vehicle, options.vehicle_params, options.vehicle_params_dir, &params_error);
        if (!params.valid)
            throw std::runtime_error("unable to load vehicle parameters: " + params_error);

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
