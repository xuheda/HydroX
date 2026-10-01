#include <string_view>
#include "hydrox/runtime/hil_runtime.h"
#include "hydrox/safety/health_manager.h"

#include <cmath>
#include <limits>
#include <utility>

namespace hydrox::runtime
{
    const char *mission_state_name(MissionState state) noexcept
    {
        switch (state)
        {
        case MissionState::RUNNING:
            return "RUNNING";
        case MissionState::COMPLETE:
            return "COMPLETE";
        case MissionState::FAILED:
            return "FAILED";
        case MissionState::IDLE:
        default:
            return "IDLE";
        }
    }

    const char *runtime_event_name(RuntimeEvent event) noexcept
    {
        switch (event)
        {
        case RuntimeEvent::SENSOR_READY:
            return "SENSOR_READY";
        case RuntimeEvent::SETPOINT_TIMEOUT:
            return "SETPOINT_TIMEOUT";
        case RuntimeEvent::SENSOR_TIMEOUT:
            return "SENSOR_TIMEOUT";
        case RuntimeEvent::SENSOR_TIME_DISCONTINUITY:
            return "SENSOR_TIME_DISCONTINUITY";
        case RuntimeEvent::CONTROL_OUTPUT_INVALID:
            return "CONTROL_OUTPUT_INVALID";
        case RuntimeEvent::SIMULATOR_PAUSED:
            return "SIMULATOR_PAUSED";
        case RuntimeEvent::DISCONNECTED:
            return "DISCONNECTED";
        case RuntimeEvent::NONE:
        default:
            return "NONE";
        }
    }

    const char *step_status_name(StepStatus status) noexcept
    {
        switch (status)
        {
        case StepStatus::CONFIGURATION_ERROR:
            return "CONFIGURATION_ERROR";
        case StepStatus::NO_SENSOR:
            return "NO_SENSOR";
        case StepStatus::NON_INCREASING_SENSOR_TIME:
            return "NON_INCREASING_SENSOR_TIME";
        case StepStatus::SENSOR_TIME_GAP:
            return "SENSOR_TIME_GAP";
        case StepStatus::OK:
        default:
            return "OK";
        }
    }

    HilRuntime::HilRuntime(HilRuntimeConfig config,
                           std::unique_ptr<IController> controller,
                           std::unique_ptr<IAllocator> allocator,
                           IWrenchAugmentor *augmentor)
        : config_(std::move(config)),
          controller_(std::move(controller)),
          allocator_(std::move(allocator)),
          augmentor_(augmentor),
          ekf_(config_.estimation_profile),
          motor_(config_.motor),
          energy_(config_.energy),
          safety_supervisor_(config_.safety_profile),
          failsafe_navigator_(config_.safety_profile),
          setpoint_(config_.initial_setpoint),
          armed_(config_.initial_armed)
    {
        if (!(config_.nominal_dt_s > 0.0) ||
            !std::isfinite(config_.nominal_dt_s))
        {
            config_.nominal_dt_s = kDefaultHilControlPeriodS;
        }
        reset_pipeline();
        initialize_safety_status(0);
    }

    bool HilRuntime::valid() const noexcept
    {
        return controller_ != nullptr && allocator_ != nullptr &&
            config_.safety_profile.valid() && config_.safety_profile.vehicle_class ==
                config_.estimation_profile.vehicle_class;
    }

    void HilRuntime::reset_pipeline()
    {
        failsafe_context_ = safety::FailsafeContext{};
        last_position_aid_us_ = last_vertical_aid_us_ = 0;
        previous_sensor_time_us_ = 0;
        last_sensor_arrival_us_ = 0;
        setpoint_ = config_.initial_setpoint;
        mode_ = GNCMode::DISABLED;
        mission_state_ = MissionState::IDLE;
        have_external_setpoint_ = false;
        session_control_ready_ = false;
        safety_candidates_ = safety::CandidateSet{};
        last_authorized_reference_ = safety::AuthorizedReference{};
        failsafe_navigator_.reset();
        failsafe_navigator_mode_ = safety::VehicleMode::Boot;
        active_control_source_ = safety::ControlSource::None;
        external_command_sequence_ = 0;
        reset_controller_on_next_step_ = true;
        const auto prior_cause = last_tick_.safety_cause;
        const auto prior_observed = last_tick_.safety_observed_s;
        const auto prior_threshold = last_tick_.safety_threshold_s;
        last_tick_ = HilRuntimeTick{};
        last_tick_.safety_status = safety_supervisor_.output();
        last_tick_.safety_cause = prior_cause;
        last_tick_.safety_observed_s = prior_observed;
        last_tick_.safety_threshold_s = prior_threshold;
        last_tick_.estimated_state = config_.initial_state;
        last_tick_.control_state = config_.initial_state;
        last_tick_.armed = armed_;

        ekf_.reset(config_.initial_state);
        motor_.reset();
        energy_.reset();
        if (augmentor_ != nullptr)
            augmentor_->reset();
        if (controller_ != nullptr)
        {
            controller_->set_mode(GNCMode::DISABLED);
            controller_->set_setpoint(setpoint_);
            controller_->reset(config_.initial_state);
        }
    }

    uint64_t HilRuntime::on_connected(platform::MonotonicTimeUs now_us)
    {
        reset_pipeline();
        const uint64_t generation = control_session_.on_connected(now_us);
        update_safety_status(now_us);
        return generation;
    }

    RuntimeEvent HilRuntime::on_disconnected(platform::MonotonicTimeUs now_us)
    {
        control_session_.on_disconnected(now_us);
        invalidate_session_authority();
        update_safety_status(now_us, RuntimeEvent::DISCONNECTED);
        inhibit_outputs();
        return RuntimeEvent::DISCONNECTED;
    }

    RuntimeEvent HilRuntime::on_simulator_paused(
        platform::MonotonicTimeUs now_us)
    {
        control_session_.on_disconnected(now_us);
        invalidate_session_authority();
        update_safety_status(now_us, RuntimeEvent::SIMULATOR_PAUSED);
        inhibit_outputs();
        return RuntimeEvent::SIMULATOR_PAUSED;
    }

    uint64_t HilRuntime::on_simulator_resumed(
        platform::MonotonicTimeUs now_us)
    {
        // Retain estimator/controller state across an intentional simulation
        // pause, but create a fresh authority epoch. A resumed sensor and a
        // newly received command are both required before outputs can arm.
        invalidate_session_authority();
        inhibit_outputs();
        const uint64_t generation = control_session_.on_connected(now_us);
        update_safety_status(now_us);
        return generation;
    }

    RuntimeEvent HilRuntime::observe_valid_sensor(
        platform::MonotonicTimeUs now_us)
    {
        if (control_session_.phase() == ControlSessionPhase::DISCONNECTED)
            return RuntimeEvent::NONE;
        last_sensor_arrival_us_ = now_us;
        return control_session_.observe_valid_sensor(now_us)
                   ? RuntimeEvent::SENSOR_READY
                   : RuntimeEvent::NONE;
    }

    bool HilRuntime::set_armed(
        bool armed,
        platform::MonotonicTimeUs now_us)
    {
        if (!armed)
        {
            armed_ = false;
            control_session_.revoke_setpoint(now_us);
            inhibit_outputs();
            update_safety_status(now_us, RuntimeEvent::NONE, true);
            return true;
        }

        const ControlSessionPhase phase = control_session_.phase();
        if (!navigation_ready(now_us) || phase == ControlSessionPhase::DISCONNECTED ||
            phase == ControlSessionPhase::WAITING_FOR_SENSOR)
        {
            return false;
        }

        // Disarming revoked the prior command epoch. Re-arming therefore never
        // replays an old setpoint; a fresh command must open the gate again.
        if (safety_supervisor_.output().mode !=
                safety::VehicleMode::Standby)
        {
            return false;
        }
        armed_ = true;
        last_tick_.armed = true;
        update_safety_status(now_us);
        if (!last_tick_.safety_status.armed)
        {
            armed_ = false;
            last_tick_.armed = false;
            return false;
        }
        return true;
    }

    bool HilRuntime::acknowledge_safety_fault(
        platform::MonotonicTimeUs now_us)
    {
        if (armed_ || !navigation_ready(now_us))
        {
            return false;
        }

        safety::SupervisorInput input{};
        input.now_us = now_us;
        input.health = assess_health(now_us).snapshot;
        input.session_suspended = !session_control_ready_;
        input.fault_acknowledged = true;
        last_tick_.safety_status = safety_supervisor_.update(input);
        const bool acknowledged = last_tick_.safety_status.mode ==
            safety::VehicleMode::Standby && !last_tick_.safety_status.operator_ack_required;
        if (acknowledged)
        {
            last_tick_.safety_cause = "FaultAcknowledged";
            last_tick_.safety_observed_s = last_tick_.safety_threshold_s = -1.0;
        }
        return acknowledged;
    }

    bool HilRuntime::accept_setpoint(
        const GNCSetpoint &setpoint,
        GNCMode mode,
        platform::MonotonicTimeUs received_at_us)
    {
        if (!valid())
            return false;
        // Account for elapsed loss before a late packet refreshes its timestamp.
        maintain(received_at_us);
        if (!valid_gnc_setpoint(setpoint, mode) || !config_.safety_profile.accepts_mode(mode))
            return false;
        if (!control_session_.accept_setpoint(received_at_us))
            return false;

        if (mode == GNCMode::DISABLED)
        {
            revoke_setpoint(received_at_us);
            return true;
        }
        session_control_ready_ = true;
        have_external_setpoint_ = true;
        safety::ControlCandidate &candidate = safety_candidates_.external;
        candidate = safety::ControlCandidate{};
        candidate.source = safety::ControlSource::External;
        candidate.session_id = control_session_.generation();
        candidate.sequence = ++external_command_sequence_;
        candidate.received_at_us = received_at_us;
        const uint64_t validity = config_.safety_profile.external_command_warn_us;
        candidate.valid_until_us =
            received_at_us > std::numeric_limits<uint64_t>::max() - validity
                ? std::numeric_limits<uint64_t>::max()
                : received_at_us + validity;
        candidate.mode = mode;
        candidate.setpoint = setpoint;
        candidate.valid = true;
        update_safety_status(received_at_us);
        return true; // Only the safety arbiter changes the running controller.
    }

    bool HilRuntime::resume_external_control(platform::MonotonicTimeUs now_us)
    {
        const auto &candidate = safety_candidates_.external;
        if (!armed_ || !navigation_ready(now_us) ||
            candidate.session_id != control_session_.generation() ||
            !candidate.valid || now_us < candidate.received_at_us ||
            now_us >= candidate.valid_until_us || !control_session_.is_active() ||
            last_sensor_arrival_us_ == 0 || now_us < last_sensor_arrival_us_ ||
            now_us - last_sensor_arrival_us_ > config_.sensor_timeout_us ||
            last_tick_.safety_navigation_degraded)
            return false;
        if (controller_->transition_fault() && !controller_->acknowledge_transition_fault())
            return false;
        safety::SupervisorInput input{};
        input.now_us = now_us;
        input.health = assess_health(now_us).snapshot;
        input.resume_requested = true;
        input.external_control_requested = true;
        input.external_command_valid = true;
        input.external_command_age_us = now_us - candidate.received_at_us;
        last_tick_.safety_status = safety_supervisor_.update(input);
        if (!last_tick_.safety_status.external_authorized) return false;
        last_tick_.safety_cause = "OperatorResume";
        last_tick_.safety_observed_s = last_tick_.safety_threshold_s = -1.0;
        have_external_setpoint_ = true;
        return true;
    }

    void HilRuntime::initialize_safety_status(
        platform::MonotonicTimeUs now_us)
    {
        safety_supervisor_.reset(now_us);
        safety::SupervisorInput input{};
        input.now_us = now_us;
        input.boot_complete = true;
        input.health = assess_health(now_us).snapshot;
        last_tick_.safety_status = safety_supervisor_.update(input);

        if (armed_)
        {
            input = safety::SupervisorInput{};
            input.now_us = now_us;
            input.health = assess_health(now_us).snapshot;
            input.arm_requested = true;
            last_tick_.safety_status = safety_supervisor_.update(input);
        }
    }

    HilRuntime::HealthAssessment HilRuntime::assess_health(
        platform::MonotonicTimeUs now_us, RuntimeEvent event) const
    {
        using namespace safety;
        // Rebuild from current runtime facts: recovered reports must not linger.
        // Fault latching belongs exclusively to VehicleSupervisor.
        const HealthMask required = health_component_bit(HealthComponent::Platform);
        HealthManager manager(required, required);
        ComponentHealth platform{};
        platform.component = HealthComponent::Platform;
        platform.observed_at_us = now_us;
        platform.healthy = platform.can_arm = platform.control_available = valid();
        manager.update(platform);

        const auto report = [&](HealthComponent component, bool control_available,
                                SafetyAction action, SafetyReason reason)
        {
            ComponentHealth health{};
            health.component = component;
            health.observed_at_us = now_us;
            health.healthy = false;
            health.can_arm = health.control_available = control_available;
            health.severity = control_available ? HealthSeverity::DEGRADED
                                                : HealthSeverity::CRITICAL;
            health.recommended_action = action;
            health.reason = reason;
            manager.update(health);
        };
        if (event == RuntimeEvent::SENSOR_TIMEOUT ||
            event == RuntimeEvent::SENSOR_TIME_DISCONTINUITY ||
            event == RuntimeEvent::CONTROL_OUTPUT_INVALID)
        {
            report(event == RuntimeEvent::CONTROL_OUTPUT_INVALID
                       ? HealthComponent::Controller : HealthComponent::Imu,
                   false, SafetyAction::DisableOutput,
                   SafetyReason::ControlCoreUnavailable);
        }

        const bool monitor_navigation = event == RuntimeEvent::NONE &&
            previous_sensor_time_us_ != 0 && armed_ &&
            (have_external_setpoint_ || safety_supervisor_.output().operator_ack_required);
        if (monitor_navigation)
        {
            if (session_control_ready_ && last_tick_.safety_navigation_degraded)
                report(HealthComponent::EstimatorPosition, true,
                       SafetyAction::Stabilize, SafetyReason::ComponentStale);
            if (session_control_ready_ && controller_ && controller_->transition_fault())
                report(HealthComponent::Controller, true,
                       SafetyAction::Stabilize, SafetyReason::ComponentFault);
            const bool finite = last_tick_.control_state.eta.allFinite() &&
                last_tick_.control_state.nu.allFinite() &&
                std::isfinite(last_tick_.control_state.depth_m);
            if (!finite)
                report(HealthComponent::EstimatorAttitude, false,
                       SafetyAction::DisableOutput, SafetyReason::ControlCoreUnavailable);
            else if (!failsafe_context_.vertical_available &&
                     config_.safety_profile.vehicle_class != VehicleClass::USV &&
                     config_.safety_profile.vehicle_class != VehicleClass::UGV_DIFFERENTIAL)
                report(HealthComponent::EstimatorDepth, false,
                       SafetyAction::DisableOutput, SafetyReason::ControlCoreUnavailable);
        }

        HealthAssessment result{};
        result.snapshot = manager.snapshot(now_us);
        result.cause = runtime_event_name(event);
        if (event == RuntimeEvent::SENSOR_TIMEOUT)
        {
            result.observed_s = now_us >= last_sensor_arrival_us_
                ? (now_us - last_sensor_arrival_us_) * 1e-6 : -1.0;
            result.threshold_s = config_.sensor_timeout_us * 1e-6;
        }
        else if (event == RuntimeEvent::SENSOR_TIME_DISCONTINUITY)
        {
            result.observed_s = last_tick_.dt;
            result.threshold_s = config_.max_sensor_dt_s;
        }
        else if (event == RuntimeEvent::SETPOINT_TIMEOUT)
        {
            result.observed_s = last_tick_.setpoint_age_s;
            result.threshold_s = config_.safety_profile.external_command_loss_us * 1e-6;
        }
        else if (event == RuntimeEvent::NONE &&
                 result.snapshot.reason != SafetyReason::None)
        {
            // Diagnostics follow the same primary report selected by HealthManager.
            switch (result.snapshot.primary_component)
            {
            case HealthComponent::EstimatorPosition:
                result.cause = "PositionAidUnavailable";
                result.observed_s = last_tick_.position_aid_age_s;
                result.threshold_s = 3.0;
                break;
            case HealthComponent::Controller:
                result.cause = "FlightPhaseTransitionFault";
                break;
            case HealthComponent::EstimatorDepth:
                result.cause = "VerticalAidUnavailable";
                result.observed_s = last_tick_.vertical_aid_age_s;
                result.threshold_s = 1.0;
                break;
            case HealthComponent::EstimatorAttitude:
                result.cause = "NonFiniteEstimatedState";
                break;
            default:
                result.cause = safety_reason_name(result.snapshot.reason);
                break;
            }
        }
        return result;
    }

    void HilRuntime::update_safety_status(
        platform::MonotonicTimeUs now_us,
        RuntimeEvent event,
        bool external_source_released)
    {
        safety::SupervisorInput input{};
        input.now_us = now_us;
        input.session_suspended = !session_control_ready_;
        input.external_source_released = external_source_released;
        input.external_control_requested = have_external_setpoint_;
        input.external_command_valid =
            have_external_setpoint_ && control_session_.is_active();

        const double age_s = control_session_.setpoint_age_s(now_us);
        if (std::isfinite(age_s) && age_s >= 0.0)
        {
            constexpr double maximum_us =
                static_cast<double>(
                    std::numeric_limits<uint64_t>::max());
            const double age_us = age_s * 1.0e6;
            input.external_command_age_us =
                age_us >= maximum_us
                    ? std::numeric_limits<uint64_t>::max()
                    : static_cast<uint64_t>(age_us);
        }

        const HealthAssessment assessment = assess_health(now_us, event);
        input.health = assessment.snapshot;
        const char *cause = external_source_released && event == RuntimeEvent::NONE
                                ? "ExternalSourceReleased" : assessment.cause;
        const double observed = assessment.observed_s;
        const double threshold = assessment.threshold_s;

        const safety::SupervisorOutput current = safety_supervisor_.output();
        input.disarm_requested = !armed_ && current.armed;
        input.arm_requested =
            armed_ && current.mode == safety::VehicleMode::Standby;
        last_tick_.safety_status = safety_supervisor_.update(input);
        if (current.transition_sequence != last_tick_.safety_status.transition_sequence)
        {
            if (event == RuntimeEvent::NONE && std::string_view(cause) == "NONE")
                cause = safety::safety_reason_name(last_tick_.safety_status.reason);
            last_tick_.safety_cause = cause;
            last_tick_.safety_observed_s = observed;
            last_tick_.safety_threshold_s = threshold;
        }
    }

    bool HilRuntime::prepare_safety_reference(
        const NavigationState &control_state,
        platform::MonotonicTimeUs now_us)
    {
        const safety::SupervisorOutput supervisor =
            last_tick_.safety_status;
        const auto is_failsafe_mode = [](safety::VehicleMode mode) noexcept
        {
            switch (mode)
            {
            case safety::VehicleMode::CommandHold:
            case safety::VehicleMode::FailsafeStabilize:
            case safety::VehicleMode::FailsafeHold:
            case safety::VehicleMode::FailsafeSurface:
            case safety::VehicleMode::EmergencyAbort:
                return true;
            default:
                return false;
            }
        };

        if (is_failsafe_mode(supervisor.mode))
        {
            if (supervisor.mode != safety::VehicleMode::CommandHold)
            {
                have_external_setpoint_ = false;
                mission_state_ = MissionState::FAILED;
            }
            if (failsafe_navigator_mode_ != supervisor.mode)
            {
                const safety::AuthorizedReference *last =
                    last_authorized_reference_.valid
                        ? &last_authorized_reference_
                        : nullptr;
                if (!failsafe_navigator_.enter(
                        supervisor.mode, control_state, last, now_us, failsafe_context_))
                {
                    safety_candidates_.failsafe =
                        safety::ControlCandidate{};
                    safety_candidates_.emergency =
                        safety::ControlCandidate{};
                }
                failsafe_navigator_mode_ = supervisor.mode;
            }

            safety::ControlCandidate candidate =
                failsafe_navigator_.update(
                    supervisor.mode, control_state, now_us, failsafe_context_);
            if (supervisor.mode == safety::VehicleMode::EmergencyAbort)
                safety_candidates_.emergency = candidate;
            else
                safety_candidates_.failsafe = candidate;
        }
        else if (failsafe_navigator_mode_ != safety::VehicleMode::Boot)
        {
            failsafe_navigator_.reset();
            failsafe_navigator_mode_ = safety::VehicleMode::Boot;
        }

        const safety::AuthorizedReference authorized =
            safety_arbiter_.select(
                supervisor, safety_candidates_, now_us);
        if (!authorized.valid || !authorized.actuator_authorized ||
            !config_.safety_profile.accepts_mode(authorized.mode) ||
            (authorized.source == safety::ControlSource::External &&
             authorized.session_id != control_session_.generation()))
        {
            if (supervisor.actuator_authorized)
                fail_safe(now_us, RuntimeEvent::CONTROL_OUTPUT_INVALID);
            active_control_source_ = safety::ControlSource::None;
            mode_ = GNCMode::DISABLED;
            setpoint_.surge_ref = 0.0;
            setpoint_.use_yaw_rate_ref = false;
            setpoint_.yaw_rate_ref = 0.0;
            controller_->set_mode(mode_);
            controller_->set_setpoint(setpoint_);
            reset_controller_on_next_step_ = true;
            return false;
        }

        const bool mode_changed = mode_ != authorized.mode;
        active_control_source_ = authorized.source;
        mode_ = authorized.mode;
        setpoint_ = authorized.setpoint;
        controller_->set_mode(mode_);
        controller_->set_setpoint(setpoint_);
        reset_controller_on_next_step_ =
            reset_controller_on_next_step_ ||
            (mode_changed && config_.safety_profile.vehicle_kind != safety::VehicleSafetyKind::Vtol);
        if (authorized.source == safety::ControlSource::External)
            mission_state_ = MissionState::RUNNING;
        last_authorized_reference_ = authorized;
        return true;
    }

    void HilRuntime::invalidate_session_authority()
    {
        session_control_ready_ = false;
        have_external_setpoint_ = false;
        safety_candidates_ = safety::CandidateSet{};
        last_authorized_reference_ = safety::AuthorizedReference{};
        failsafe_navigator_.reset();
        failsafe_navigator_mode_ = safety::VehicleMode::Boot;
        previous_sensor_time_us_ = 0;
        last_sensor_arrival_us_ = 0;
        last_position_aid_us_ = last_vertical_aid_us_ = 0;
        failsafe_context_ = safety::FailsafeContext{};
    }

    bool HilRuntime::navigation_ready(platform::MonotonicTimeUs now_us) const
    {
        if (!valid() || previous_sensor_time_us_ == 0 ||
            last_sensor_arrival_us_ == 0 || now_us < last_sensor_arrival_us_ ||
            now_us - last_sensor_arrival_us_ > config_.sensor_timeout_us ||
            !last_tick_.control_state.eta.allFinite() ||
            !last_tick_.control_state.nu.allFinite() ||
            !std::isfinite(last_tick_.control_state.depth_m))
            return false;
        if (last_tick_.used_truth)
            return true;
        const auto domain = config_.safety_profile.vehicle_class;
        const bool needs_position = domain == VehicleClass::UAV_MULTIROTOR ||
            domain == VehicleClass::UAV_FIXED_WING || domain == VehicleClass::UAV_VTOL;
        const bool needs_vertical = domain != VehicleClass::USV &&
            domain != VehicleClass::UGV_DIFFERENTIAL;
        return (!needs_position || (last_position_aid_us_ != 0 &&
                    now_us >= last_position_aid_us_ && now_us - last_position_aid_us_ <= 3'000'000)) &&
               (!needs_vertical || (last_vertical_aid_us_ != 0 &&
                    now_us >= last_vertical_aid_us_ && now_us - last_vertical_aid_us_ <= 1'000'000));
    }

    void HilRuntime::inhibit_outputs()
    {
        have_external_setpoint_ = false;
        mode_ = GNCMode::DISABLED;
        mission_state_ = last_tick_.safety_status.operator_ack_required
            ? MissionState::FAILED : MissionState::IDLE;
        active_control_source_ = safety::ControlSource::None;
        last_tick_.safety_control_source = safety::ControlSource::None;
        setpoint_.surge_ref = 0.0;
        setpoint_.use_yaw_rate_ref = false;
        setpoint_.yaw_rate_ref = 0.0;
        reset_controller_on_next_step_ = true;
        last_tick_.wrench.setZero();
        last_tick_.actuator = ActuatorCmd{};
        last_tick_.armed = armed_;
        last_tick_.actuator_authorized = false;
        last_tick_.actuator_mode = kMavModeFlagHilEnabled;
        last_tick_.mode = mode_;
        last_tick_.mission_state = mission_state_;
        last_tick_.have_external_setpoint = false;
        last_tick_.setpoint_age_s = -1.0;
        if (controller_ != nullptr)
        {
            controller_->set_mode(mode_);
            controller_->set_setpoint(setpoint_);
        }
        if (augmentor_ != nullptr)
            augmentor_->reset();
    }

    RuntimeEvent HilRuntime::fail_safe(
        platform::MonotonicTimeUs now_us,
        RuntimeEvent event)
    {
        last_tick_.setpoint_age_s = control_session_.setpoint_age_s(now_us);
        control_session_.revoke_setpoint(now_us);
        have_external_setpoint_ = false;
        safety_candidates_.external = safety::ControlCandidate{};
        update_safety_status(now_us, event, false);
        if (!last_tick_.safety_status.actuator_authorized)
            inhibit_outputs();
        return event;
    }

    RuntimeEvent HilRuntime::revoke_setpoint(
        platform::MonotonicTimeUs now_us)
    {
        control_session_.revoke_setpoint(now_us);
        have_external_setpoint_ = false;
        safety_candidates_.external = safety::ControlCandidate{};
        update_safety_status(now_us, RuntimeEvent::NONE, true);
        if (!last_tick_.safety_status.actuator_authorized)
            inhibit_outputs();
        return RuntimeEvent::NONE;
    }

    RuntimeEvent HilRuntime::maintain(platform::MonotonicTimeUs now_us)
    {
        // Sensor loss takes priority, including while an onboard fallback owns control.
        if (config_.sensor_timeout_us > 0 && session_control_ready_ &&
            last_sensor_arrival_us_ > 0 && now_us > last_sensor_arrival_us_ &&
            now_us - last_sensor_arrival_us_ > config_.sensor_timeout_us &&
            last_tick_.safety_status.mode != safety::VehicleMode::OutputDisabled &&
            last_tick_.safety_status.mode != safety::VehicleMode::FaultLocked)
            return fail_safe(now_us, RuntimeEvent::SENSOR_TIMEOUT);
        if (control_session_.setpoint_timed_out(now_us, config_.safety_profile.external_command_loss_us))
            return fail_safe(now_us, RuntimeEvent::SETPOINT_TIMEOUT);
        update_safety_status(now_us);
        if (!last_tick_.safety_status.actuator_authorized)
            inhibit_outputs();
        return RuntimeEvent::NONE;
    }

    StepStatus HilRuntime::step(const NavigationInput &input,
                                platform::MonotonicTimeUs now_us)
    {
        if (!valid())
            return StepStatus::CONFIGURATION_ERROR;
        if (control_session_.phase() == ControlSessionPhase::DISCONNECTED ||
            !input.got_imu || input.imu.time_usec == 0)
            return StepStatus::NO_SENSOR;
        if (previous_sensor_time_us_ != 0 &&
            input.imu.time_usec <= previous_sensor_time_us_)
        {
            return StepStatus::NON_INCREASING_SENSOR_TIME;
        }

        const RuntimeEvent sensor_event = observe_valid_sensor(now_us);
        (void)sensor_event;

        double dt = config_.nominal_dt_s;
        if (previous_sensor_time_us_ != 0)
        {
            dt = static_cast<double>(
                     input.imu.time_usec - previous_sensor_time_us_) *
                 1e-6;
        }
        previous_sensor_time_us_ = input.imu.time_usec;

        if (!std::isfinite(dt) || !(dt > 0.0) ||
            (config_.max_sensor_dt_s > 0.0 &&
             dt > config_.max_sensor_dt_s))
        {
            last_tick_.dt = dt; // Retain the rejected sample gap for diagnostics.
            fail_safe(now_us, RuntimeEvent::SENSOR_TIME_DISCONTINUITY);
            return StepStatus::SENSOR_TIME_GAP;
        }

        NavigationMeasurements measurements = input.measurements;
        if (!config_.allow_truth_heading_aid)
            measurements.truth_heading_debug.meta.valid = false;

        last_tick_.estimated_state = ekf_.update(
            measurements,
            input.dvl,
            input.water_dvl,
            input.gps,
            dt);
        last_tick_.estimated_state.dvl_valid = input.dvl_recent;
        last_tick_.estimated_state.airspeed_valid = measurements.airspeed.meta.valid;
        last_tick_.estimated_state.equivalent_airspeed_mps = measurements.airspeed.value;

        const ControlFeedbackSelection feedback = select_control_feedback(
            last_tick_.estimated_state,
            input.truth,
            input.truth_valid,
            config_.control_feedback_source);
        last_tick_.control_state = feedback.state;

        const auto &stats = ekf_.last_stats();
        if (stats.gps_xy_accepted > 0)
            last_position_aid_us_ = now_us;
        if (stats.depth_accepted > 0 || stats.gps_z_accepted > 0)
            last_vertical_aid_us_ = now_us;
        failsafe_context_.position_available = last_position_aid_us_ != 0 &&
            now_us >= last_position_aid_us_ && now_us - last_position_aid_us_ <= 3'000'000;
        failsafe_context_.vertical_available = last_vertical_aid_us_ != 0 &&
            now_us >= last_vertical_aid_us_ && now_us - last_vertical_aid_us_ <= 1'000'000;
        if (feedback.used_truth)
        {
            failsafe_context_.position_available = true;
            failsafe_context_.vertical_available = true;
        }
        last_tick_.position_aid_age_s = last_position_aid_us_ && now_us >= last_position_aid_us_
            ? (now_us - last_position_aid_us_) * 1e-6 : -1.0;
        last_tick_.vertical_aid_age_s = last_vertical_aid_us_ && now_us >= last_vertical_aid_us_
            ? (now_us - last_vertical_aid_us_) * 1e-6 : -1.0;
        last_tick_.position_available = failsafe_context_.position_available;
        last_tick_.vertical_available = failsafe_context_.vertical_available;
        failsafe_context_.flight_phase = controller_->flight_phase();
        const auto domain = config_.safety_profile.vehicle_class;
        const bool needs_position = domain == VehicleClass::UAV_MULTIROTOR ||
            domain == VehicleClass::UAV_FIXED_WING || domain == VehicleClass::UAV_VTOL ||
            (domain == VehicleClass::USV && safety_candidates_.external.mode == GNCMode::WAYPOINT_3D);
        const bool needs_vertical = domain != VehicleClass::USV && domain != VehicleClass::UGV_DIFFERENTIAL;
        last_tick_.safety_navigation_degraded =
            (needs_position && !failsafe_context_.position_available) ||
            (needs_vertical && !failsafe_context_.vertical_available);

        update_safety_status(now_us);

        bool authorized = prepare_safety_reference(last_tick_.control_state, now_us);

        bool controller_reset = false;
        if (reset_controller_on_next_step_)
        {
            controller_->reset(last_tick_.control_state);
            reset_controller_on_next_step_ = false;
            controller_reset = true;
        }

        const ActuatorCmd previous_actuator = last_tick_.actuator;
        last_tick_.base_wrench.setZero();
        last_tick_.augmentor_candidate_wrench.setZero();
        last_tick_.wrench.setZero();
        last_tick_.wrench_augmentor_observed = false;
        last_tick_.wrench_augmentor_candidate_valid = false;
        last_tick_.wrench_augmentor_active = false;
        last_tick_.actuator = ActuatorCmd{};
        if (authorized)
        {
            const Wrench base = controller_->update(
                last_tick_.control_state, dt);
            last_tick_.base_wrench = base;
            last_tick_.wrench = base;
            if (augmentor_ != nullptr)
            {
                last_tick_.wrench_augmentor_observed = true;
                const Wrench candidate = augmentor_->update(
                    last_tick_.estimated_state, setpoint_, base,
                    previous_actuator, dt);
                last_tick_.augmentor_candidate_wrench = candidate;
                last_tick_.wrench_augmentor_candidate_valid =
                    candidate.allFinite();
                if (config_.wrench_augmentor_active_enabled &&
                    active_control_source_ == safety::ControlSource::External &&
                    last_tick_.wrench_augmentor_candidate_valid)
                {
                    last_tick_.wrench = candidate;
                    last_tick_.wrench_augmentor_active = true;
                }
            }
            last_tick_.actuator = allocator_->allocate(
                last_tick_.wrench,
                config_.estimation_profile.vehicle_class == VehicleClass::UAV_FIXED_WING ||
                config_.estimation_profile.vehicle_class == VehicleClass::UAV_VTOL
                    ? (last_tick_.control_state.airspeed_valid
                        ? last_tick_.control_state.equivalent_airspeed_mps : 0.0)
                    : last_tick_.control_state.surge());
            // Never arm an actuator packet whose units/layout contract is
            // incomplete or whose normalized values are non-finite/out of range.
            if (!last_tick_.base_wrench.allFinite() || !last_tick_.wrench.allFinite() ||
                !last_tick_.actuator.valid())
            {
                fail_safe(now_us, RuntimeEvent::CONTROL_OUTPUT_INVALID);
                authorized = false;
            }
        }

        last_tick_.motor = motor_.step(last_tick_.actuator.rpm, dt);
        last_tick_.energy = energy_.update(last_tick_.motor.power_W, dt);

        last_tick_.waypoint_distance_m = -1.0;
        if (mode_ == GNCMode::WAYPOINT_3D)
        {
            const double dn = last_tick_.control_state.eta[0] - setpoint_.wp_n;
            const double de = last_tick_.control_state.eta[1] - setpoint_.wp_e;
            const double dd = last_tick_.control_state.eta[2] - setpoint_.wp_d;
            last_tick_.waypoint_distance_m =
                std::sqrt(dn * dn + de * de + dd * dd);
        }

        if (last_tick_.safety_status.operator_ack_required)
            mission_state_ = MissionState::FAILED;
        else if (mode_ == GNCMode::DISABLED)
            mission_state_ = MissionState::IDLE;
        else if (mission_state_ == MissionState::IDLE)
            mission_state_ = MissionState::RUNNING;
        else if (mission_state_ == MissionState::RUNNING &&
                 last_tick_.waypoint_distance_m >= 0.0 &&
                 last_tick_.waypoint_distance_m <= config_.mission_radius_m)
            mission_state_ = MissionState::COMPLETE;

        ++last_tick_.tick;
        last_tick_.sensor_time_us = input.imu.time_usec;
        last_tick_.dt = dt;
        last_tick_.setpoint_age_s =
            control_session_.setpoint_age_s(now_us);
        last_tick_.mode = mode_;
        last_tick_.mission_state = mission_state_;
        last_tick_.used_truth = feedback.used_truth;
        last_tick_.ekf_initialized = true;
        last_tick_.controller_reset = controller_reset;
        last_tick_.have_external_setpoint = have_external_setpoint_;
        last_tick_.armed = armed_;
        last_tick_.actuator_authorized = authorized;
        last_tick_.actuator_mode = static_cast<uint8_t>(
            kMavModeFlagHilEnabled |
            (authorized ? kMavModeFlagSafetyArmed : 0));
        last_tick_.safety_control_source = authorized ? active_control_source_ : safety::ControlSource::None;
        return StepStatus::OK;
    }
} // namespace hydrox::runtime
