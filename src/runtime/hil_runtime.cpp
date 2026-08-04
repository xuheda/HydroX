#include "hydrox/runtime/hil_runtime.h"

#include <cmath>
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
          setpoint_(config_.initial_setpoint)
    {
        if (!(config_.nominal_dt_s > 0.0) ||
            !std::isfinite(config_.nominal_dt_s))
        {
            config_.nominal_dt_s = 0.01;
        }
        reset_pipeline();
    }

    bool HilRuntime::valid() const noexcept
    {
        return controller_ != nullptr && allocator_ != nullptr;
    }

    void HilRuntime::reset_pipeline()
    {
        previous_sensor_time_us_ = 0;
        last_sensor_arrival_us_ = 0;
        setpoint_ = config_.initial_setpoint;
        mode_ = GNCMode::DISABLED;
        mission_state_ = MissionState::IDLE;
        have_external_setpoint_ = false;
        reset_controller_on_next_step_ = true;
        last_tick_ = HilRuntimeTick{};
        last_tick_.estimated_state = config_.initial_state;
        last_tick_.control_state = config_.initial_state;

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
        return control_session_.on_connected(now_us);
    }

    RuntimeEvent HilRuntime::on_disconnected(platform::MonotonicTimeUs now_us)
    {
        control_session_.on_disconnected(now_us);
        enter_safe_state();
        return RuntimeEvent::DISCONNECTED;
    }

    RuntimeEvent HilRuntime::observe_valid_sensor(
        platform::MonotonicTimeUs now_us)
    {
        last_sensor_arrival_us_ = now_us;
        return control_session_.observe_valid_sensor(now_us)
                   ? RuntimeEvent::SENSOR_READY
                   : RuntimeEvent::NONE;
    }

    bool HilRuntime::accept_setpoint(
        const GNCSetpoint &setpoint,
        GNCMode mode,
        platform::MonotonicTimeUs received_at_us)
    {
        if (!valid())
            return false;
        if (!control_session_.accept_setpoint(received_at_us))
            return false;

        const GNCMode previous_mode = mode_;
        setpoint_ = setpoint;
        mode_ = mode;
        have_external_setpoint_ = true;
        mission_state_ = mode_ == GNCMode::DISABLED
                             ? MissionState::IDLE
                             : MissionState::RUNNING;
        reset_controller_on_next_step_ =
            reset_controller_on_next_step_ ||
            (mode_ != GNCMode::DISABLED && previous_mode != mode_);
        controller_->set_mode(mode_);
        controller_->set_setpoint(setpoint_);
        return true;
    }

    void HilRuntime::enter_safe_state()
    {
        have_external_setpoint_ = false;
        mode_ = GNCMode::DISABLED;
        mission_state_ = MissionState::IDLE;
        setpoint_.surge_ref = 0.0;
        setpoint_.use_yaw_rate_ref = false;
        setpoint_.yaw_rate_ref = 0.0;
        reset_controller_on_next_step_ = true;
        last_tick_.wrench.setZero();
        last_tick_.actuator = ActuatorCmd{};
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
        control_session_.revoke_setpoint(now_us);
        enter_safe_state();
        return event;
    }

    RuntimeEvent HilRuntime::revoke_setpoint(
        platform::MonotonicTimeUs now_us)
    {
        return fail_safe(now_us, RuntimeEvent::NONE);
    }

    RuntimeEvent HilRuntime::maintain(platform::MonotonicTimeUs now_us)
    {
        if (control_session_.setpoint_timed_out(
                now_us, config_.setpoint_timeout_us))
        {
            return fail_safe(now_us, RuntimeEvent::SETPOINT_TIMEOUT);
        }

        if (config_.sensor_timeout_us > 0 &&
            control_session_.is_active() &&
            last_sensor_arrival_us_ > 0 &&
            now_us > last_sensor_arrival_us_ &&
            now_us - last_sensor_arrival_us_ > config_.sensor_timeout_us)
        {
            return fail_safe(now_us, RuntimeEvent::SENSOR_TIMEOUT);
        }
        return RuntimeEvent::NONE;
    }

    StepStatus HilRuntime::step(const NavigationInput &input,
                                platform::MonotonicTimeUs now_us)
    {
        if (!valid())
            return StepStatus::CONFIGURATION_ERROR;
        if (!input.got_imu || input.imu.time_usec == 0)
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

        const ControlFeedbackSelection feedback = select_control_feedback(
            last_tick_.estimated_state,
            input.truth,
            input.truth_valid,
            config_.control_feedback_source);
        last_tick_.control_state = feedback.state;

        if (reset_controller_on_next_step_)
        {
            controller_->reset(last_tick_.control_state);
            reset_controller_on_next_step_ = false;
        }

        const bool authorized = control_session_.is_active() &&
                                have_external_setpoint_ &&
                                mode_ != GNCMode::DISABLED;
        last_tick_.wrench.setZero();
        last_tick_.actuator = ActuatorCmd{};
        if (authorized)
        {
            const Wrench base = controller_->update(
                last_tick_.control_state, dt);
            last_tick_.wrench = augmentor_ != nullptr
                                    ? augmentor_->update(
                                          last_tick_.estimated_state,
                                          setpoint_, base, dt)
                                    : base;
            last_tick_.actuator = allocator_->allocate(
                last_tick_.wrench,
                last_tick_.control_state.surge());
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

        if (mode_ == GNCMode::DISABLED)
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
        last_tick_.have_external_setpoint = have_external_setpoint_;
        last_tick_.actuator_authorized = authorized;
        last_tick_.actuator_mode = static_cast<uint8_t>(
            kMavModeFlagHilEnabled |
            (authorized ? kMavModeFlagSafetyArmed : 0));
        return StepStatus::OK;
    }
} // namespace hydrox::runtime
