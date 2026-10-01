#include "hydrox/safety/command_arbiter.h"
#include "hydrox/safety/failsafe_navigator.h"
#include "hydrox/safety/health_manager.h"
#include "hydrox/safety/vehicle_supervisor.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace
{
    using namespace hydrox;
    using namespace hydrox::safety;

    void require(bool condition, const std::string &message)
    {
        if (!condition)
        {
            std::cerr << "FAIL: " << message << '\n';
            std::exit(1);
        }
    }

    void require_near(double actual,
                      double expected,
                      double tolerance,
                      const std::string &message)
    {
        require(std::abs(actual - expected) <= tolerance,
                message + " actual=" + std::to_string(actual) +
                    " expected=" + std::to_string(expected));
    }

    VehicleHealthSnapshot healthy_vehicle()
    {
        VehicleHealthSnapshot health{};
        health.can_arm = true;
        health.control_available = true;
        return health;
    }

    SupervisorOutput enter_external_control(VehicleSupervisor &supervisor)
    {
        SupervisorInput input{};
        input.health = healthy_vehicle();
        input.boot_complete = true;
        auto output = supervisor.update(input);
        require(output.mode == VehicleMode::Standby,
                "boot checks should enter standby");

        input = SupervisorInput{};
        input.now_us = 10;
        input.health = healthy_vehicle();
        input.arm_requested = true;
        output = supervisor.update(input);
        require(output.mode == VehicleMode::ArmedIdle && output.armed,
                "operator arm should enter armed idle");

        input = SupervisorInput{};
        input.now_us = 20;
        input.health = healthy_vehicle();
        input.external_control_requested = true;
        input.external_command_valid = true;
        input.external_command_age_us = 0;
        output = supervisor.update(input);
        require(output.mode == VehicleMode::ExternalControl &&
                    output.external_authorized &&
                    output.actuator_authorized,
                "fresh external command should be authorized");
        return output;
    }

    void test_health_manager()
    {
        const HealthMask required =
            health_component_bit(HealthComponent::Imu) |
            health_component_bit(HealthComponent::DepthSensor);
        HealthManager manager(required, required, SafetyAction::Stabilize);

        ComponentHealth imu{};
        imu.component = HealthComponent::Imu;
        imu.observed_at_us = 100;
        imu.stale_after_us = 100;
        require(manager.update(imu), "valid IMU health report accepted");

        auto health = manager.snapshot(150);
        require(!health.can_arm && !health.control_available,
                "missing required depth report blocks arming and control");
        require(health.reason == SafetyReason::ComponentMissing,
                "missing required component is diagnosed");
        require(health.recommended_action == SafetyAction::Stabilize,
                "missing control component requests configured fallback");

        ComponentHealth depth{};
        depth.component = HealthComponent::DepthSensor;
        depth.observed_at_us = 100;
        depth.stale_after_us = 100;
        require(manager.update(depth), "valid depth report accepted");
        health = manager.snapshot(150);
        require(health.can_arm && health.control_available,
                "fresh required reports permit arming and control");

        health = manager.snapshot(201);
        require(!health.control_available,
                "stale required health report removes control availability");
        require((health.stale_components &
                 health_component_bit(HealthComponent::Imu)) != 0,
                "stale component mask identifies IMU");
        require(health.reason == SafetyReason::ComponentStale,
                "stale report has an explicit reason");
    }

    void test_health_priority_and_recovery()
    {
        HealthManager manager;
        ComponentHealth position{};
        position.component = HealthComponent::EstimatorPosition;
        position.healthy = false;
        position.severity = HealthSeverity::DEGRADED;
        position.recommended_action = SafetyAction::Stabilize;
        position.reason = SafetyReason::ComponentStale;
        manager.update(position);
        auto health = manager.snapshot(100);
        require(health.control_available && health.reason == SafetyReason::ComponentStale,
                "navigation degradation preserves bounded attitude/vertical control");
        ComponentHealth vertical{};
        vertical.component = HealthComponent::EstimatorDepth;
        vertical.healthy = vertical.can_arm = vertical.control_available = false;
        vertical.severity = HealthSeverity::CRITICAL;
        vertical.recommended_action = SafetyAction::DisableOutput;
        vertical.reason = SafetyReason::ControlCoreUnavailable;
        manager.update(vertical);
        health = manager.snapshot(100);
        require(!health.can_arm && !health.control_available &&
                    health.primary_component == HealthComponent::EstimatorDepth &&
                    health.reason == SafetyReason::ControlCoreUnavailable &&
                    health.recommended_action == SafetyAction::DisableOutput,
                "fatal vertical loss wins over position degradation");
        require((health.unhealthy_components & health_component_bit(HealthComponent::EstimatorPosition)) &&
                    (health.unhealthy_components & health_component_bit(HealthComponent::EstimatorDepth)),
                "aggregation retains both component failures");
        manager.reset();
        ComponentHealth recovered{};
        recovered.component = HealthComponent::EstimatorDepth;
        manager.update(recovered);
        health = manager.snapshot(200);
        require(health.can_arm && health.control_available &&
                    health.unhealthy_components == 0 && health.reason == SafetyReason::None,
                "aggregator reports current facts; supervisor alone owns fault latches");
    }

    void test_supervisor_external_loss_sequence()
    {
        VehicleSupervisor supervisor;
        enter_external_control(supervisor);

        SupervisorInput input{};
        input.now_us = 200'020;
        input.health = healthy_vehicle();
        input.external_command_valid = true;
        input.external_command_age_us = 200'000;
        auto output = supervisor.update(input);
        require(output.mode == VehicleMode::CommandHold,
                "warning timeout enters bounded command hold");
        require(output.action == SafetyAction::HoldCommand && output.armed,
                "command hold keeps onboard control active");

        input.now_us = 250'020;
        input.external_command_valid = true;
        input.external_command_age_us = 0;
        output = supervisor.update(input);
        require(output.mode == VehicleMode::ExternalControl,
                "brief packet gap may recover before hard fallback");

        input.now_us = 500'020;
        input.external_command_age_us = 250'000;
        output = supervisor.update(input);
        require(output.mode == VehicleMode::CommandHold,
                "second stale interval enters command hold");

        input.now_us = 800'020;
        input.external_command_valid = false;
        input.external_command_age_us = 550'000;
        output = supervisor.update(input);
        require(output.mode == VehicleMode::FailsafeStabilize,
                "hard command loss enters onboard stabilization");
        require(output.actuator_authorized && output.operator_ack_required,
                "hard fallback keeps control authority and latches acknowledgement");

        input.now_us = 3'800'020;
        input.external_control_requested = true;
        input.external_command_valid = true;
        input.external_command_age_us = 0;
        output = supervisor.update(input);
        require(output.mode == VehicleMode::FailsafeSurface,
                "stabilize dwell escalates to configured controlled surfacing");
        require(output.action == SafetyAction::ControlledSurface,
                "surfacing action is explicit");

        input.now_us = 3'900'020;
        output = supervisor.update(input);
        require(output.mode == VehicleMode::FailsafeSurface,
                "fresh external packets cannot auto-recover a latched fallback");

        input = SupervisorInput{};
        input.now_us = 4'000'020;
        input.health = healthy_vehicle();
        input.disarm_requested = true;
        output = supervisor.update(input);
        require(output.mode == VehicleMode::FaultLocked && !output.armed,
                "operator disarm ends fallback in a locked safe state");

        input = SupervisorInput{};
        input.now_us = 4'100'020;
        input.health = healthy_vehicle();
        input.fault_acknowledged = true;
        output = supervisor.update(input);
        require(output.mode == VehicleMode::Standby &&
                    !output.operator_ack_required,
                "healthy acknowledged fault returns to standby, not armed mode");
    }

    ControlCandidate valid_candidate(ControlSource source,
                                     uint64_t now_us,
                                     uint64_t sequence)
    {
        ControlCandidate candidate{};
        candidate.source = source;
        candidate.sequence = sequence;
        candidate.received_at_us = now_us;
        candidate.valid_until_us = now_us + 100'000;
        candidate.mode = GNCMode::DEPTH_HOLD;
        candidate.setpoint.depth_ref = 20.0;
        candidate.setpoint.heading_ref = 0.5;
        candidate.setpoint.surge_ref = 2.0;
        candidate.valid = true;
        return candidate;
    }

    void test_command_arbiter()
    {
        CommandArbiter arbiter;
        CandidateSet candidates{};
        candidates.failsafe =
            valid_candidate(ControlSource::Failsafe, 100, 999);
        candidates.external =
            valid_candidate(ControlSource::External, 100, 7);

        SupervisorOutput supervisor{};
        supervisor.mode = VehicleMode::ExternalControl;
        supervisor.armed = true;
        supervisor.external_authorized = true;
        supervisor.actuator_authorized = true;
        auto authorized = arbiter.select(supervisor, candidates, 100);
        require(authorized.valid &&
                    authorized.source == ControlSource::External &&
                    authorized.sequence == 7,
                "mode ownership wins; another candidate's sequence cannot steal authority");

        supervisor.mode = VehicleMode::FailsafeStabilize;
        authorized = arbiter.select(supervisor, candidates, 100);
        require(authorized.valid && authorized.source == ControlSource::Failsafe &&
                    authorized.sequence == 999,
                "failsafe mode exclusively selects its own candidate");
        supervisor.mode = VehicleMode::ExternalControl;
        candidates.external.source = ControlSource::Failsafe;
        authorized = arbiter.select(supervisor, candidates, 100);
        require(!authorized.valid, "a mismatched source cannot use the external slot");
        candidates.external.source = ControlSource::External;

        authorized = arbiter.select(supervisor, candidates, 200'000);
        require(!authorized.valid &&
                    authorized.reason == SafetyReason::InvalidReference,
                "expired candidate is rejected at the final arbitration gate");

        supervisor.actuator_authorized = false;
        authorized = arbiter.select(supervisor, candidates, 100);
        require(!authorized.valid && !authorized.actuator_authorized,
                "supervisor output gate cannot be bypassed by a valid command");
    }

    void test_slender_auv_failsafe_navigator()
    {
        SafetyProfile profile{};
        VehicleFailsafeNavigator navigator(profile);
        const FailsafeContext context{true, true, FlightPhase::NotApplicable};
        NavigationState state = NavigationState::zeros();
        state.depth_m = 20.0;
        state.eta[5] = 1.0;
        state.nu[0] = 5.0;

        AuthorizedReference last{};
        last.valid = true;
        last.mode = GNCMode::DEPTH_HOLD;
        last.setpoint.depth_ref = 20.0;
        last.setpoint.heading_ref = 1.0;
        last.setpoint.surge_ref = 5.0;
        last.setpoint.use_yaw_rate_ref = true;
        last.setpoint.yaw_rate_ref = 1.0;

        require(navigator.enter(
                    VehicleMode::CommandHold, state, &last, 0, context),
                "command-hold navigator requires and accepts last authority");
        auto candidate = navigator.update(
            VehicleMode::CommandHold, state, 100'000, context);
        require(candidate.valid && candidate.setpoint.use_yaw_rate_ref,
                "short hold produces a valid onboard reference");
        require_near(candidate.setpoint.yaw_rate_ref,
                     0.95,
                     1.0e-9,
                     "yaw rate slews toward zero instead of freezing the turn");
        require_near(candidate.setpoint.surge_ref,
                     5.0,
                     1.0e-9,
                     "short hold does not abruptly cut propulsion");

        require(navigator.enter(
                    VehicleMode::FailsafeStabilize, state, nullptr, 100'000, context),
                "stabilize mode captures a safe reference");
        candidate = navigator.update(
            VehicleMode::FailsafeStabilize, state, 300'000, context);
        require(candidate.valid && candidate.mode == GNCMode::DEPTH_HOLD,
                "stabilization uses depth and heading hold");
        require_near(candidate.setpoint.surge_ref,
                     4.9,
                     1.0e-9,
                     "surge decelerates with the configured slew limit");
        require(candidate.setpoint.surge_ref >=
                    profile.minimum_control_surge_mps,
                "stern-control authority is retained while stabilizing");

        require(navigator.enter(
                    VehicleMode::FailsafeSurface, state, nullptr, 300'000, context),
                "surfacing mode starts from current depth");
        candidate = navigator.update(
            VehicleMode::FailsafeSurface, state, 500'000, context);
        require(candidate.valid && candidate.mode == GNCMode::DEPTH_HOLD,
                "deep vehicle surfaces through a moving depth reference");
        require_near(candidate.setpoint.depth_ref,
                     19.94,
                     1.0e-9,
                     "surface depth reference moves upward at a bounded rate");
        require(candidate.setpoint.surge_ref > 0.0,
                "controlled surfacing does not cut thrust while deeply submerged");

        state.depth_m = 0.2;
        candidate = navigator.update(
            VehicleMode::FailsafeSurface, state, 700'000, context);
        require(candidate.valid && candidate.mode == GNCMode::SURFACE,
                "near-surface vehicle transitions to the surface controller mode");
    }
} // namespace

int main()
{
    test_health_manager();
    test_health_priority_and_recovery();
    test_supervisor_external_loss_sequence();
    test_command_arbiter();
    test_slender_auv_failsafe_navigator();
    std::cout << "PASS: safety architecture behavior\n";
    return 0;
}
