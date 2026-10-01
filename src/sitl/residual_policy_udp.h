#pragma once

#include "learning/residual_rl.h"
#include "sitl/sitl_platform.h"

#include <cstdint>
#include <memory>
#include <string>

namespace hydrox::sitl
{
    constexpr uint32_t kResidualPolicyRequestMagic = 0x48585251U;  // HXRQ
    constexpr uint32_t kResidualPolicyResponseMagic = 0x48585253U; // HXRS
    constexpr uint16_t kResidualPolicyWireVersion = 1;
    constexpr uint32_t kResidualPolicyResponseValid = 1U << 0;
    constexpr uint32_t kResidualPolicyModelReady = 1U << 1;
    constexpr uint32_t kResidualPolicyFinite = 1U << 2;
    constexpr uint32_t kResidualPolicyRequiredFlags =
        kResidualPolicyResponseValid |
        kResidualPolicyModelReady |
        kResidualPolicyFinite;

#pragma pack(push, 1)
    struct ResidualPolicyRequestV1
    {
        uint32_t magic = kResidualPolicyRequestMagic;
        uint16_t version = kResidualPolicyWireVersion;
        uint16_t size = sizeof(ResidualPolicyRequestV1);
        uint64_t nonce = 0;
        uint64_t sequence = 0;
        uint32_t flags = 0; // bit 0: reset causal policy state
        uint32_t reserved = 0;
        double dt_s = 0.0;
        double eta[6]{};
        double nu[6]{};
        double depth_m = 0.0;
        double current_body_uv[2]{};
        // depth, heading, surge, use_yaw_rate, yaw_rate, wp_n/e/d,
        // use_path_segment, path_start_n/e, lookahead, arrival, hold_heading.
        double setpoint[14]{};
        double base_x_m_n[3]{};
        double previous_delta_x_n[2]{};
        // Previous propeller command and yaw-fin command.
        double previous_actuator[2]{};
    };

    struct ResidualPolicyResponseV1
    {
        uint32_t magic = kResidualPolicyResponseMagic;
        uint16_t version = kResidualPolicyWireVersion;
        uint16_t size = sizeof(ResidualPolicyResponseV1);
        uint64_t nonce = 0;
        uint64_t sequence = 0;
        uint32_t flags = 0;
        uint32_t reserved = 0;
        double normalized_action_x_n[2]{};
        double confidence = 0.0;
        double inference_ms = 0.0;
        double pinn_ood_feature = 0.0;
        double selector_fraction_x_n[2]{};
    };
#pragma pack(pop)

    static_assert(sizeof(ResidualPolicyRequestV1) == 328);
    static_assert(sizeof(ResidualPolicyResponseV1) == 88);

    struct UdpResidualPolicyConfig
    {
        std::string host = "127.0.0.1";
        uint16_t remote_port = 0;
        uint16_t local_port = 0;
        uint64_t nonce = 0;
        int control_hz = 100;
        int policy_hz = 10;
        double timeout_ms = 5.0;
    };

    struct UdpResidualPolicyStats
    {
        uint64_t requests = 0;
        uint64_t accepted = 0;
        uint64_t timeouts = 0;
        uint64_t rejected_packets = 0;
        uint64_t transport_errors = 0;
        double last_inference_ms = 0.0;
        double last_pinn_ood_feature = 0.0;
        double last_selector_fraction_x = 0.0;
        double last_selector_fraction_n = 0.0;
    };

    /** Local-only, deadline-bounded residual policy client.

        The class is deliberately synchronous only at the policy rate. Between
        policy decisions it holds the last accepted normalized action. Any
        timeout, malformed packet, nonce/sequence mismatch, non-finite value,
        or out-of-range action returns an invalid action so the C++ safety
        filter removes learned authority.
     */
    class UdpResidualPolicy final : public learning::IResidualPolicy
    {
    public:
        explicit UdpResidualPolicy(UdpResidualPolicyConfig config);

        bool ready() const noexcept { return ready_; }
        void reset() override;
        learning::ResidualAction infer(
            const learning::ResidualObservation &observation) override;

        const UdpResidualPolicyStats &stats() const noexcept { return stats_; }

    private:
        ResidualPolicyRequestV1 make_request(
            const learning::ResidualObservation &observation,
            uint64_t sequence) const;
        bool accept_response(
            const ResidualPolicyResponseV1 &response,
            uint64_t sequence,
            learning::ResidualAction &action);

        UdpResidualPolicyConfig config_;
        std::unique_ptr<UdpSender> socket_;
        bool ready_ = false;
        bool reset_pending_ = true;
        uint64_t control_tick_ = 0;
        uint64_t sequence_ = 0;
        int stride_ = 1;
        learning::ResidualAction held_{};
        UdpResidualPolicyStats stats_{};
    };

} // namespace hydrox::sitl
