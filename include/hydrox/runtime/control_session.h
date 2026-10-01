#pragma once

#include "hydrox/platform/clock.h"

#include <algorithm>
#include <cstdint>
#include <limits>

namespace hydrox::runtime
{
    enum class ControlSessionPhase : uint8_t
    {
        DISCONNECTED = 0,
        WAITING_FOR_SENSOR,
        WAITING_FOR_SETPOINT,
        ACTIVE,
    };

    inline const char *control_session_phase_name(ControlSessionPhase phase) noexcept
    {
        switch (phase)
        {
        case ControlSessionPhase::DISCONNECTED:
            return "DISCONNECTED";
        case ControlSessionPhase::WAITING_FOR_SENSOR:
            return "WAITING_FOR_SENSOR";
        case ControlSessionPhase::WAITING_FOR_SETPOINT:
            return "WAITING_FOR_SETPOINT";
        case ControlSessionPhase::ACTIVE:
            return "ACTIVE";
        }
        return "UNKNOWN";
    }

    inline platform::MonotonicTimeUs duration_us_from_seconds(
        double seconds) noexcept
    {
        if (!(seconds > 0.0))
            return 0;

        constexpr double kMaximumSeconds =
            static_cast<double>(
                std::numeric_limits<platform::MonotonicTimeUs>::max()) *
            1e-6;
        if (seconds >= kMaximumSeconds)
            return std::numeric_limits<platform::MonotonicTimeUs>::max();

        return static_cast<platform::MonotonicTimeUs>(seconds * 1e6);
    }

    /**
     * Guards command authority across transport connection epochs.
     *
     * A connection creates a new state epoch. Commands are rejected until a
     * valid sensor sample from that epoch arrives, and commands from an older
     * epoch can never reactivate control after reconnect or revocation.
     *
     * All timestamps use platform::Clock's monotonic microsecond domain.
     */
    class ControlSessionGate
    {
    public:
        uint64_t on_connected(platform::MonotonicTimeUs now_us) noexcept
        {
            if (generation_ != std::numeric_limits<uint64_t>::max())
                ++generation_;
            phase_ = ControlSessionPhase::WAITING_FOR_SENSOR;
            setpoint_not_before_us_ = now_us;
            have_setpoint_ = false;
            return generation_;
        }

        void on_disconnected(platform::MonotonicTimeUs now_us) noexcept
        {
            phase_ = ControlSessionPhase::DISCONNECTED;
            setpoint_not_before_us_ = now_us;
            have_setpoint_ = false;
        }

        bool observe_valid_sensor(platform::MonotonicTimeUs now_us) noexcept
        {
            if (phase_ != ControlSessionPhase::WAITING_FOR_SENSOR)
                return false;

            phase_ = ControlSessionPhase::WAITING_FOR_SETPOINT;
            setpoint_not_before_us_ =
                std::max(setpoint_not_before_us_, now_us);
            return true;
        }

        bool accept_setpoint(platform::MonotonicTimeUs received_at_us) noexcept
        {
            if (phase_ != ControlSessionPhase::WAITING_FOR_SETPOINT &&
                phase_ != ControlSessionPhase::ACTIVE)
            {
                return false;
            }
            if (received_at_us < setpoint_not_before_us_)
                return false;
            if (have_setpoint_ && received_at_us < last_setpoint_received_us_)
                return false;

            last_setpoint_received_us_ = received_at_us;
            have_setpoint_ = true;
            phase_ = ControlSessionPhase::ACTIVE;
            return true;
        }

        void revoke_setpoint(platform::MonotonicTimeUs now_us) noexcept
        {
            have_setpoint_ = false;
            setpoint_not_before_us_ =
                std::max(setpoint_not_before_us_, now_us);
            if (phase_ != ControlSessionPhase::DISCONNECTED &&
                phase_ != ControlSessionPhase::WAITING_FOR_SENSOR)
            {
                phase_ = ControlSessionPhase::WAITING_FOR_SETPOINT;
            }
        }

        bool setpoint_timed_out(
            platform::MonotonicTimeUs now_us,
            platform::MonotonicTimeUs timeout_us) const noexcept
        {
            return timeout_us > 0 &&
                   phase_ == ControlSessionPhase::ACTIVE &&
                   have_setpoint_ &&
                   now_us > last_setpoint_received_us_ &&
                   now_us - last_setpoint_received_us_ > timeout_us;
        }

        double setpoint_age_s(
            platform::MonotonicTimeUs now_us) const noexcept
        {
            if (!have_setpoint_)
                return -1.0;
            if (now_us <= last_setpoint_received_us_)
                return 0.0;
            return static_cast<double>(now_us - last_setpoint_received_us_) *
                   1e-6;
        }

        uint64_t generation() const noexcept
        {
            return generation_;
        }

        ControlSessionPhase phase() const noexcept
        {
            return phase_;
        }

        bool is_active() const noexcept
        {
            return phase_ == ControlSessionPhase::ACTIVE;
        }

    private:
        uint64_t generation_ = 0;
        ControlSessionPhase phase_ = ControlSessionPhase::DISCONNECTED;
        platform::MonotonicTimeUs setpoint_not_before_us_ = 0;
        platform::MonotonicTimeUs last_setpoint_received_us_ = 0;
        bool have_setpoint_ = false;
    };
}
