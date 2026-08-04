#pragma once

#include "ekf.h"
#include "gnc/control_interfaces.h"
#include "gnc/energy_model.h"
#include "gnc/motor_model.h"
#include "hydrox/platform/clock.h"
#include "hydrox/runtime/control_feedback.h"
#include "hydrox/runtime/control_session.h"
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
        double nominal_dt_s = 0.01;
        double max_sensor_dt_s = 0.25;
        double mission_radius_m = 3.0;
        platform::MonotonicTimeUs setpoint_timeout_us = 2'000'000;
        platform::MonotonicTimeUs sensor_timeout_us = 500'000;
    };

    struct HilRuntimeTick
    {
        NavigationState estimated_state = NavigationState::zeros();
        NavigationState control_state = NavigationState::zeros();
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
        bool have_external_setpoint = false;
        bool actuator_authorized = false;
        uint8_t actuator_mode = kMavModeFlagHilEnabled;
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
        RuntimeEvent observe_valid_sensor(platform::MonotonicTimeUs now_us);
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
        MissionState mission_state() const noexcept { return mission_state_; }
        const ControlSessionGate &control_session() const noexcept
        {
            return control_session_;
        }
        const EKF &ekf() const noexcept { return ekf_; }

    private:
        void reset_pipeline();
        void enter_safe_state();
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
        GNCSetpoint setpoint_{};
        GNCMode mode_ = GNCMode::DISABLED;
        MissionState mission_state_ = MissionState::IDLE;
        HilRuntimeTick last_tick_{};
        uint64_t previous_sensor_time_us_ = 0;
        platform::MonotonicTimeUs last_sensor_arrival_us_ = 0;
        bool reset_controller_on_next_step_ = true;
        bool have_external_setpoint_ = false;
    };
} // namespace hydrox::runtime
