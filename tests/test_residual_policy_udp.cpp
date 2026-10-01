#include "sitl/residual_policy_udp.h"
#include "sitl/sitl_platform.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

namespace
{
    int fail(const char *message)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        return 1;
    }

    int run_external(int argc, char **argv)
    {
        using namespace hydrox;
        using namespace hydrox::learning;
        using namespace hydrox::sitl;
        if (argc != 5)
            return fail("external usage: external SERVER_PORT CLIENT_PORT NONCE");
        NetworkRuntime network;
        if (!network.ready())
            return fail("network runtime unavailable");
        UdpResidualPolicyConfig config;
        config.remote_port = static_cast<uint16_t>(std::strtoul(argv[2], nullptr, 10));
        config.local_port = static_cast<uint16_t>(std::strtoul(argv[3], nullptr, 10));
        config.nonce = static_cast<uint64_t>(std::strtoull(argv[4], nullptr, 10));
        config.control_hz = 100;
        config.policy_hz = 10;
        config.timeout_ms = 50.0;
        UdpResidualPolicy policy(config);
        if (!policy.ready())
            return fail("external loopback client did not initialize");

        ResidualObservation observation;
        observation.state.eta[2] = 100.0;
        observation.state.depth_m = 100.0;
        observation.state.nu[0] = 1.5;
        observation.setpoint.depth_ref = 100.0;
        observation.setpoint.heading_ref = 0.0;
        observation.setpoint.surge_ref = 1.5;
        observation.setpoint.use_yaw_rate_ref = true;
        observation.setpoint.wp_n = 5.0;
        observation.setpoint.wp_d = 100.0;
        observation.setpoint.lookahead_m = 5.0;
        observation.setpoint.arrival_radius_m = 1.0;
        observation.setpoint.hold_heading = true;
        observation.base_wrench[0] = 100.0;

        const ResidualAction first = policy.infer(observation);
        if (!first.valid || !std::isfinite(first.normalized[0]) ||
            !std::isfinite(first.normalized[5]) ||
            std::abs(first.normalized[0]) > 0.500001 ||
            std::abs(first.normalized[5]) > 0.500001)
        {
            return fail("external sidecar first response was invalid");
        }
        for (int index = 0; index < 9; ++index)
        {
            const ResidualAction held = policy.infer(observation);
            if (!held.valid ||
                std::abs(held.normalized[0] - first.normalized[0]) > 1e-12 ||
                std::abs(held.normalized[5] - first.normalized[5]) > 1e-12)
            {
                return fail("external sidecar hold contract failed");
            }
        }
        const ResidualAction second = policy.infer(observation);
        if (!second.valid || policy.stats().requests != 2 ||
            policy.stats().accepted != 2 || policy.stats().timeouts != 0 ||
            policy.stats().transport_errors != 0)
        {
            return fail("external sidecar second response or statistics failed");
        }
        std::printf(
            "external sidecar passed: first=(%.9f,%.9f) second=(%.9f,%.9f) "
            "inference_ms=%.4f ood=%.6f selector=(%.6f,%.6f)\n",
            first.normalized[0], first.normalized[5],
            second.normalized[0], second.normalized[5],
            policy.stats().last_inference_ms,
            policy.stats().last_pinn_ood_feature,
            policy.stats().last_selector_fraction_x,
            policy.stats().last_selector_fraction_n);
        return 0;
    }
}

int main(int argc, char **argv)
{
    if (argc > 1)
    {
        if (std::string(argv[1]) == "external")
            return run_external(argc, argv);
        return fail("unknown test mode");
    }
    using namespace hydrox;
    using namespace hydrox::learning;
    using namespace hydrox::sitl;
    using namespace std::chrono_literals;

    NetworkRuntime network;
    if (!network.ready())
        return fail("network runtime unavailable");

    constexpr uint16_t server_port = 47851;
    constexpr uint16_t client_port = 47852;
    constexpr uint64_t nonce = 0xEC0A900000000025ULL;
    UdpSender server("127.0.0.1", client_port, false);
    if (!server.is_open() || !server.bind_local("127.0.0.1", server_port))
        return fail("could not bind loopback test server");

    std::atomic<int> served{0};
    std::atomic<bool> request_contract_valid{true};
    std::thread responder([&]()
    {
        const auto deadline = std::chrono::steady_clock::now() + 2s;
        while (served.load() < 2 && std::chrono::steady_clock::now() < deadline)
        {
            ResidualPolicyRequestV1 request;
            const int received = server.receive(&request, sizeof(request));
            if (received == static_cast<int>(sizeof(request)))
            {
                const bool valid =
                    request.magic == kResidualPolicyRequestMagic &&
                    request.version == kResidualPolicyWireVersion &&
                    request.size == sizeof(request) &&
                    request.nonce == nonce &&
                    request.sequence == static_cast<uint64_t>(served.load() + 1) &&
                    std::abs(request.base_x_m_n[0] - 10.0) < 1e-12 &&
                    std::abs(request.previous_actuator[0] - 0.25) < 1e-6;
                request_contract_valid.store(
                    request_contract_valid.load() && valid);
                server.accept_last_peer();
                ResidualPolicyResponseV1 response;
                response.nonce = request.nonce;
                response.sequence = request.sequence;
                response.flags = kResidualPolicyRequiredFlags;
                response.normalized_action_x_n[0] = 0.25;
                response.normalized_action_x_n[1] = -0.50;
                response.confidence = 0.95;
                response.inference_ms = 1.25;
                response.pinn_ood_feature = 0.02;
                response.selector_fraction_x_n[0] = 1.0;
                response.selector_fraction_x_n[1] = 0.5;
                if (server.send(&response, sizeof(response)))
                    ++served;
            }
            else
            {
                std::this_thread::sleep_for(100us);
            }
        }
    });

    UdpResidualPolicyConfig config;
    config.remote_port = server_port;
    config.local_port = client_port;
    config.nonce = nonce;
    config.control_hz = 100;
    config.policy_hz = 10;
    config.timeout_ms = 20.0;
    UdpResidualPolicy policy(config);
    if (!policy.ready())
    {
        responder.join();
        return fail("valid loopback client was rejected");
    }

    ResidualObservation observation;
    observation.base_wrench[0] = 10.0;
    observation.previous_actuator.ch[4] = 0.25f;
    observation.previous_actuator.ch[2] = 0.2f;
    observation.previous_actuator.ch[3] = -0.2f;
    observation.state.medium_velocity_valid = true;
    observation.state.medium_velocity_kind = MediumVelocityKind::WaterCurrent;
    observation.state.medium_velocity_ned = Eigen::Vector3d(0.1, -0.2, 0.0);
    const ResidualAction first = policy.infer(observation);
    if (!first.valid ||
        std::abs(first.normalized[0] - 0.25) > 1e-12 ||
        std::abs(first.normalized[5] + 0.50) > 1e-12)
    {
        responder.join();
        return fail("valid sidecar response was not accepted");
    }
    for (int index = 0; index < 9; ++index)
    {
        const ResidualAction held = policy.infer(observation);
        if (!held.valid || std::abs(held.normalized[0] - 0.25) > 1e-12)
        {
            responder.join();
            return fail("accepted action was not held between 10 Hz decisions");
        }
    }
    const ResidualAction second = policy.infer(observation);
    responder.join();
    if (!second.valid || served.load() != 2 || !request_contract_valid.load())
        return fail("second policy decision or wire contract failed");
    if (policy.stats().requests != 2 || policy.stats().accepted != 2 ||
        policy.stats().timeouts != 0 ||
        std::abs(policy.stats().last_inference_ms - 1.25) > 1e-12)
    {
        return fail("sidecar statistics do not match accepted traffic");
    }

    UdpResidualPolicyConfig timeout_config = config;
    timeout_config.remote_port = 47853;
    timeout_config.local_port = 47854;
    timeout_config.timeout_ms = 2.0;
    UdpResidualPolicy timeout_policy(timeout_config);
    if (!timeout_policy.ready())
        return fail("timeout client socket did not initialize");
    const auto before = std::chrono::steady_clock::now();
    const ResidualAction timeout_action = timeout_policy.infer(observation);
    const auto elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - before).count();
    if (timeout_action.valid ||
        timeout_policy.stats().timeouts +
            timeout_policy.stats().transport_errors != 1 ||
        elapsed > 20.0)
    {
        return fail("missing sidecar did not fail closed within its deadline");
    }

    UdpResidualPolicyConfig invalid_config = config;
    invalid_config.host = "0.0.0.0";
    UdpResidualPolicy invalid_policy(invalid_config);
    if (invalid_policy.ready() || invalid_policy.infer(observation).valid)
        return fail("non-loopback residual policy transport was accepted");

    std::puts("test_residual_policy_udp: all checks passed");
    return 0;
}
