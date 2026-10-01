#pragma once

#include "ekf.h"
#include "gnc/control_interfaces.h"
#include "gnc/energy_model.h"
#include "gnc/motor_model.h"
#include "hydrox/platform/clock.h"
#include "hydrox/runtime/control_feedback.h"
#include "hydrox/runtime/control_session.h"
#include "hydrox/runtime/hil_contract.h"
#include "hydrox/safety/command_arbiter.h"
#include "hydrox/safety/failsafe_navigator.h"
#include "hydrox/safety/vehicle_supervisor.h"
#include "sensor_adapter.h"

#include <cstdint>
#include <memory>

namespace hydrox::runtime
{
    constexpr uint8_t kMavModeFlagHilEnabled = 32;
    constexpr uint8_t kMavModeFlagSafetyArmed = 128;

    enum class MissionState : uint8_t
    {
        IDLE = 0,
        RUNNING = 1,
        COMPLETE = 2,
        FAILED = 3,
    };

    const char *mission_state_name(MissionState state) noexcept;

    enum class RuntimeEvent : uint8_t
    {
        NONE = 0,
        SENSOR_READY,
        SETPOINT_TIMEOUT,
        SENSOR_TIMEOUT,
        SENSOR_TIME_DISCONTINUITY,
        CONTROL_OUTPUT_INVALID,
        SIMULATOR_PAUSED,
        DISCONNECTED,
    };

    const char *runtime_event_name(RuntimeEvent event) noexcept;

    enum class StepStatus : uint8_t
    {
        OK = 0,
        CONFIGURATION_ERROR,
        NO_SENSOR,
        NON_INCREASING_SENSOR_TIME,
        SENSOR_TIME_GAP,
    };

    const char *step_status_name(StepStatus status) noexcept;

    /** Optional, bounded-authority hook executed inside the shared pipeline. */
    class IWrenchAugmentor
    {
    public:
        virtual ~IWrenchAugmentor() = default;
        virtual void reset() = 0;
        virtual Wrench update(const NavigationState &estimated_state,
                              const GNCSetpoint &setpoint,
                              const Wrench &base_wrench,
                              const ActuatorCmd &previous_actuator,
                              double dt) = 0;
    };

    struct HilRuntimeConfig
    {
        EstimationProfile estimation_profile =
            estimation_profile_for(VehicleClass::UUV);
        NavigationState initial_state = NavigationState::zeros();
        GNCSetpoint initial_setpoint{};
        MotorModel::Params motor{};
        EnergyModel::Params energy{};
        ControlFeedbackSource control_feedback_source =
            ControlFeedbackSource::EstimatedState;
        bool allow_truth_heading_aid = false;
        double nominal_dt_s = kDefaultHilControlPeriodS;
        double max_sensor_dt_s = kDefaultHilMaxSensorDtS;
        double mission_radius_m = 3.0;
        platform::MonotonicTimeUs sensor_timeout_us =
            kDefaultHilSensorTimeoutUs;
        // Simulated startup arming; physical output inhibition is a board contract.
        bool initial_armed = true;
        // Evaluate an installed wrench augmentor without granting it actuator
        // authority by default.  Active authority is an explicit, independent
        // deployment gate; a missing or invalid candidate always falls back
        // to the classical controller wrench.
        bool wrench_augmentor_active_enabled = false;
        safety::SafetyProfile safety_profile{};
    };

    struct HilRuntimeTick
    {
        NavigationState estimated_state = NavigationState::zeros();
        NavigationState control_state = NavigationState::zeros();
        Wrench base_wrench = Wrench::Zero();
        Wrench augmentor_candidate_wrench = Wrench::Zero();
        Wrench wrench = Wrench::Zero();
        ActuatorCmd actuator{};
        MotorState motor{};
        EnergyState energy{};
        uint64_t sensor_time_us = 0;
        uint32_t tick = 0;
        double dt = 0.0;
        double waypoint_distance_m = -1.0;
        double setpoint_age_s = -1.0;
        GNCMode mode = GNCMode::DISABLED;
        MissionState mission_state = MissionState::IDLE;
        bool used_truth = false;
        bool ekf_initialized = false;
        bool controller_reset = false;
        bool have_external_setpoint = false;
        bool armed = false;
        bool actuator_authorized = false;
        bool wrench_augmentor_observed = false;
        bool wrench_augmentor_candidate_valid = false;
        bool wrench_augmentor_active = false;
        bool safety_navigation_degraded = false;
        uint8_t actuator_mode = kMavModeFlagHilEnabled;
        // Diagnostic only; never consumed by the controller or safety arbiter.
        const char* safety_cause = "None";
        double safety_observed_s = -1.0, safety_threshold_s = -1.0;
        double position_aid_age_s = -1.0, vertical_aid_age_s = -1.0;
        bool position_available = false, vertical_available = false;
        safety::SupervisorOutput safety_status{};
        safety::ControlSource safety_control_source =
            safety::ControlSource::None;
    };

    /**
     * Platform-independent HIL flight pipeline shared by SITL and HITL.
     *
     * Transports and parameter storage live outside this class. Once a
     * NavigationInput and a fresh command enter the class, estimator, GNC,
     * allocation, arming and failsafe semantics are identical on every target.
     */
    class HilRuntime
    {
    public:
        HilRuntime(HilRuntimeConfig config,
                   std::unique_ptr<IController> controller,
                   std::unique_ptr<IAllocator> allocator,
                   IWrenchAugmentor *augmentor = nullptr);

        bool valid() const noexcept;

        uint64_t on_connected(platform::MonotonicTimeUs now_us);
        RuntimeEvent on_disconnected(platform::MonotonicTimeUs now_us);
        RuntimeEvent on_simulator_paused(platform::MonotonicTimeUs now_us);
        uint64_t on_simulator_resumed(platform::MonotonicTimeUs now_us);
        RuntimeEvent observe_valid_sensor(platform::MonotonicTimeUs now_us);
        bool set_armed(bool armed, platform::MonotonicTimeUs now_us);
        bool acknowledge_safety_fault(
            platform::MonotonicTimeUs now_us);
        bool resume_external_control(platform::MonotonicTimeUs now_us);
        bool accept_setpoint(const GNCSetpoint &setpoint,
                             GNCMode mode,
                             platform::MonotonicTimeUs received_at_us);
        RuntimeEvent revoke_setpoint(platform::MonotonicTimeUs now_us);
        RuntimeEvent maintain(platform::MonotonicTimeUs now_us);
        StepStatus step(const NavigationInput &input,
                        platform::MonotonicTimeUs now_us);

        const HilRuntimeConfig &config() const noexcept { return config_; }
        const HilRuntimeTick &last_tick() const noexcept { return last_tick_; }
        const GNCSetpoint &setpoint() const noexcept { return setpoint_; }
        GNCMode mode() const noexcept { return mode_; }
        bool armed() const noexcept { return armed_; }
        MissionState mission_state() const noexcept { return mission_state_; }
        const ControlSessionGate &control_session() const noexcept
        {
            return control_session_;
        }
        const EKF &ekf() const noexcept { return ekf_; }
        const safety::SupervisorOutput &safety_status() const noexcept
        {
            return last_tick_.safety_status;
        }

    private:
        struct HealthAssessment
        {
            safety::VehicleHealthSnapshot snapshot{};
            const char *cause = "NONE";
            double observed_s = -1.0;
            double threshold_s = -1.0;
        };
        HealthAssessment assess_health(platform::MonotonicTimeUs now_us,
                                       RuntimeEvent event = RuntimeEvent::NONE) const;
        void reset_pipeline();
        void inhibit_outputs();
        void invalidate_session_authority();
        bool navigation_ready(platform::MonotonicTimeUs now_us) const;
        void initialize_safety_status(platform::MonotonicTimeUs now_us);
        void update_safety_status(platform::MonotonicTimeUs now_us,
                                  RuntimeEvent event = RuntimeEvent::NONE,
                                  bool external_source_released = false);
        bool prepare_safety_reference(
            const NavigationState &control_state,
            platform::MonotonicTimeUs now_us);
        RuntimeEvent fail_safe(platform::MonotonicTimeUs now_us,
                               RuntimeEvent event);

        HilRuntimeConfig config_;
        std::unique_ptr<IController> controller_;
        std::unique_ptr<IAllocator> allocator_;
        IWrenchAugmentor *augmentor_ = nullptr;
        EKF ekf_;
        MotorModel motor_;
        EnergyModel energy_;
        ControlSessionGate control_session_;
        safety::VehicleSupervisor safety_supervisor_;
        safety::CommandArbiter safety_arbiter_;
        safety::VehicleFailsafeNavigator failsafe_navigator_;
        safety::CandidateSet safety_candidates_{};
        safety::AuthorizedReference last_authorized_reference_{};
        safety::VehicleMode failsafe_navigator_mode_ =
            safety::VehicleMode::Boot;
        safety::ControlSource active_control_source_ =
            safety::ControlSource::None;
        GNCSetpoint setpoint_{};
        GNCMode mode_ = GNCMode::DISABLED;
        MissionState mission_state_ = MissionState::IDLE;
        HilRuntimeTick last_tick_{};
        uint64_t previous_sensor_time_us_ = 0;
        platform::MonotonicTimeUs last_sensor_arrival_us_ = 0;
        bool reset_controller_on_next_step_ = true;
        bool have_external_setpoint_ = false;
        bool armed_ = false;
        bool session_control_ready_ = false;
        uint64_t external_command_sequence_ = 0;
        safety::FailsafeContext failsafe_context_{};
        uint64_t last_position_aid_us_ = 0;
        uint64_t last_vertical_aid_us_ = 0;
    };
} // namespace hydrox::runtime
