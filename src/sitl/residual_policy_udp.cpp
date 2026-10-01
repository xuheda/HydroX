#include "sitl/residual_policy_udp.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace hydrox::sitl
{
namespace
{
    bool finite_response(const ResidualPolicyResponseV1 &response)
    {
        return std::isfinite(response.normalized_action_x_n[0]) &&
            std::isfinite(response.normalized_action_x_n[1]) &&
            std::isfinite(response.confidence) &&
            std::isfinite(response.inference_ms) &&
            std::isfinite(response.pinn_ood_feature) &&
            std::isfinite(response.selector_fraction_x_n[0]) &&
            std::isfinite(response.selector_fraction_x_n[1]);
    }
}

UdpResidualPolicy::UdpResidualPolicy(UdpResidualPolicyConfig config)
    : config_(std::move(config))
{
    const bool valid =
        config_.host == "127.0.0.1" &&
        config_.remote_port != 0 &&
        config_.local_port != 0 &&
        config_.remote_port != config_.local_port &&
        config_.nonce != 0 &&
        config_.control_hz > 0 &&
        config_.policy_hz > 0 &&
        config_.control_hz % config_.policy_hz == 0 &&
        std::isfinite(config_.timeout_ms) &&
        config_.timeout_ms > 0.0 &&
        config_.timeout_ms < 1000.0 / config_.policy_hz;
    if (!valid)
        return;
    stride_ = config_.control_hz / config_.policy_hz;
    socket_ = std::make_unique<UdpSender>(
        config_.host, config_.remote_port, false);
    ready_ = socket_->is_open() &&
        socket_->bind_local("127.0.0.1", config_.local_port);
    if (!ready_)
        socket_.reset();
}

void UdpResidualPolicy::reset()
{
    reset_pending_ = true;
    control_tick_ = 0;
    sequence_ = 0;
    held_ = learning::ResidualAction{};
}

ResidualPolicyRequestV1 UdpResidualPolicy::make_request(
    const learning::ResidualObservation &observation,
    uint64_t sequence) const
{
    ResidualPolicyRequestV1 request;
    request.nonce = config_.nonce;
    request.sequence = sequence;
    request.flags = reset_pending_ ? 1U : 0U;
    request.dt_s = 1.0 / static_cast<double>(config_.policy_hz);
    for (int index = 0; index < 6; ++index)
    {
        request.eta[index] = observation.state.eta[index];
        request.nu[index] = observation.state.nu[index];
    }
    request.depth_m = observation.state.depth_m;
    if (observation.state.medium_velocity_valid &&
        observation.state.medium_velocity_kind ==
            MediumVelocityKind::WaterCurrent)
    {
        const double yaw = observation.state.eta[5];
        const double north = observation.state.medium_velocity_ned[0];
        const double east = observation.state.medium_velocity_ned[1];
        request.current_body_uv[0] =
            std::cos(yaw) * north + std::sin(yaw) * east;
        request.current_body_uv[1] =
            -std::sin(yaw) * north + std::cos(yaw) * east;
    }
    const GNCSetpoint &setpoint = observation.setpoint;
    request.setpoint[0] = setpoint.depth_ref;
    request.setpoint[1] = setpoint.heading_ref;
    request.setpoint[2] = setpoint.surge_ref;
    request.setpoint[3] = setpoint.use_yaw_rate_ref ? 1.0 : 0.0;
    request.setpoint[4] = setpoint.yaw_rate_ref;
    request.setpoint[5] = setpoint.wp_n;
    request.setpoint[6] = setpoint.wp_e;
    request.setpoint[7] = setpoint.wp_d;
    request.setpoint[8] = setpoint.use_path_segment ? 1.0 : 0.0;
    request.setpoint[9] = setpoint.path_start_n;
    request.setpoint[10] = setpoint.path_start_e;
    request.setpoint[11] = setpoint.lookahead_m;
    request.setpoint[12] = setpoint.arrival_radius_m;
    request.setpoint[13] = setpoint.hold_heading ? 1.0 : 0.0;
    request.base_x_m_n[0] = observation.base_wrench[0];
    request.base_x_m_n[1] = observation.base_wrench[4];
    request.base_x_m_n[2] = observation.base_wrench[5];
    request.previous_delta_x_n[0] = observation.previous_applied_delta[0];
    request.previous_delta_x_n[1] = observation.previous_applied_delta[5];
    request.previous_actuator[0] = observation.previous_actuator.ch[4];
    request.previous_actuator[1] = 0.5 * (
        observation.previous_actuator.ch[2] -
        observation.previous_actuator.ch[3]);
    return request;
}

bool UdpResidualPolicy::accept_response(
    const ResidualPolicyResponseV1 &response,
    uint64_t sequence,
    learning::ResidualAction &action)
{
    const bool valid =
        response.magic == kResidualPolicyResponseMagic &&
        response.version == kResidualPolicyWireVersion &&
        response.size == sizeof(ResidualPolicyResponseV1) &&
        response.nonce == config_.nonce &&
        response.sequence == sequence &&
        (response.flags & kResidualPolicyRequiredFlags) ==
            kResidualPolicyRequiredFlags &&
        finite_response(response) &&
        std::abs(response.normalized_action_x_n[0]) <= 1.0 &&
        std::abs(response.normalized_action_x_n[1]) <= 1.0 &&
        response.confidence >= 0.0 && response.confidence <= 1.0 &&
        response.inference_ms >= 0.0 &&
        response.pinn_ood_feature >= 0.0 &&
        response.selector_fraction_x_n[0] >= 0.0 &&
        response.selector_fraction_x_n[0] <= 1.0 &&
        response.selector_fraction_x_n[1] >= 0.0 &&
        response.selector_fraction_x_n[1] <= 1.0;
    if (!valid)
        return false;
    action = learning::ResidualAction{};
    action.normalized[0] = response.normalized_action_x_n[0];
    action.normalized[5] = response.normalized_action_x_n[1];
    action.confidence = response.confidence;
    action.valid = true;
    stats_.last_inference_ms = response.inference_ms;
    stats_.last_pinn_ood_feature = response.pinn_ood_feature;
    stats_.last_selector_fraction_x = response.selector_fraction_x_n[0];
    stats_.last_selector_fraction_n = response.selector_fraction_x_n[1];
    return true;
}

learning::ResidualAction UdpResidualPolicy::infer(
    const learning::ResidualObservation &observation)
{
    if (!ready_ || socket_ == nullptr)
        return {};
    const uint64_t tick = control_tick_++;
    if (tick % static_cast<uint64_t>(stride_) != 0)
        return held_;

    const uint64_t sequence = ++sequence_;
    const ResidualPolicyRequestV1 request = make_request(
        observation, sequence);
    ++stats_.requests;
    if (!socket_->send(&request, sizeof(request)))
    {
        ++stats_.transport_errors;
        held_ = {};
        return held_;
    }

    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double, std::milli>(config_.timeout_ms);
    while (std::chrono::steady_clock::now() < deadline)
    {
        ResidualPolicyResponseV1 response;
        const int received = socket_->receive(&response, sizeof(response));
        if (received < 0)
        {
            ++stats_.transport_errors;
            held_ = {};
            return held_;
        }
        if (received == static_cast<int>(sizeof(response)))
        {
            learning::ResidualAction candidate;
            if (accept_response(response, sequence, candidate))
            {
                held_ = candidate;
                reset_pending_ = false;
                ++stats_.accepted;
                return held_;
            }
            ++stats_.rejected_packets;
        }
        else if (received > 0)
        {
            ++stats_.rejected_packets;
        }
        // Windows' default scheduler quantum can turn a microsecond sleep into
        // a 15+ ms stall. Yield keeps the bounded policy deadline precise;
        // this loop only runs once per 10 Hz policy decision.
        std::this_thread::yield();
    }
    ++stats_.timeouts;
    held_ = {};
    return held_;
}

} // namespace hydrox::sitl
