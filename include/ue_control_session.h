#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <optional>

namespace hydrox::sitl
{
    enum class UeControlSessionPhase : uint8_t
    {
        DISCONNECTED = 0,
        WAITING_FOR_SENSOR,
        WAITING_FOR_SETPOINT,
        ACTIVE,
    };

    inline const char *ue_control_session_phase_name(UeControlSessionPhase phase)
    {
        switch (phase)
        {
        case UeControlSessionPhase::DISCONNECTED:
            return "DISCONNECTED";
        case UeControlSessionPhase::WAITING_FOR_SENSOR:
            return "WAITING_FOR_SENSOR";
        case UeControlSessionPhase::WAITING_FOR_SETPOINT:
            return "WAITING_FOR_SETPOINT";
        case UeControlSessionPhase::ACTIVE:
            return "ACTIVE";
        }
        return "UNKNOWN";
    }

    /**
     * Gates external control across UE simulator connection epochs.
     *
     * A reconnect creates a new byte stream and a new simulated-state epoch.
     * Commands received before the first valid sensor sample of that epoch are
     * never allowed to reactivate control. Setpoint age is based exclusively on
     * steady_clock, so simulator time pause or rollback cannot extend a lease.
     */
    class UeControlSessionGate
    {
    public:
        using Clock = std::chrono::steady_clock;
        using TimePoint = Clock::time_point;

        uint64_t on_connected(TimePoint now)
        {
            ++generation_;
            phase_ = UeControlSessionPhase::WAITING_FOR_SENSOR;
            setpoint_not_before_ = now;
            last_setpoint_received_at_.reset();
            return generation_;
        }

        void on_disconnected(TimePoint now)
        {
            phase_ = UeControlSessionPhase::DISCONNECTED;
            setpoint_not_before_ = now;
            last_setpoint_received_at_.reset();
        }

        bool observe_valid_sensor(TimePoint now)
        {
            if (phase_ != UeControlSessionPhase::WAITING_FOR_SENSOR)
                return false;

            phase_ = UeControlSessionPhase::WAITING_FOR_SETPOINT;
            setpoint_not_before_ = std::max(setpoint_not_before_, now);
            return true;
        }

        bool accept_setpoint(TimePoint received_at)
        {
            if (phase_ != UeControlSessionPhase::WAITING_FOR_SETPOINT &&
                phase_ != UeControlSessionPhase::ACTIVE)
            {
                return false;
            }
            if (received_at < setpoint_not_before_)
                return false;
            if (last_setpoint_received_at_ &&
                received_at < *last_setpoint_received_at_)
            {
                return false;
            }

            last_setpoint_received_at_ = received_at;
            phase_ = UeControlSessionPhase::ACTIVE;
            return true;
        }

        void revoke_setpoint(TimePoint now)
        {
            last_setpoint_received_at_.reset();
            setpoint_not_before_ = std::max(setpoint_not_before_, now);
            if (phase_ != UeControlSessionPhase::DISCONNECTED &&
                phase_ != UeControlSessionPhase::WAITING_FOR_SENSOR)
            {
                phase_ = UeControlSessionPhase::WAITING_FOR_SETPOINT;
            }
        }

        bool setpoint_timed_out(TimePoint now,
                                std::chrono::duration<double> timeout) const
        {
            return timeout.count() > 0.0 &&
                   phase_ == UeControlSessionPhase::ACTIVE &&
                   last_setpoint_received_at_ &&
                   now - *last_setpoint_received_at_ > timeout;
        }

        double setpoint_age_s(TimePoint now) const
        {
            if (!last_setpoint_received_at_)
                return -1.0;
            return std::max(
                0.0,
                std::chrono::duration<double>(now - *last_setpoint_received_at_)
                    .count());
        }

        uint64_t generation() const { return generation_; }
        UeControlSessionPhase phase() const { return phase_; }
        bool is_active() const
        {
            return phase_ == UeControlSessionPhase::ACTIVE;
        }

    private:
        uint64_t generation_ = 0;
        UeControlSessionPhase phase_ = UeControlSessionPhase::DISCONNECTED;
        TimePoint setpoint_not_before_{};
        std::optional<TimePoint> last_setpoint_received_at_;
    };
} // namespace hydrox::sitl
