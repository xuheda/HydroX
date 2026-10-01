#pragma once

#include <chrono>
#include <cstdint>

namespace hydrox::sitl
{
    enum class DdsConnectionState : uint8_t
    {
        DISCONNECTED = 0,
        CONNECTING,
        CONNECTED,
        BACKOFF,
        STOPPED,
    };

    struct DdsConnectionStatus
    {
        DdsConnectionState state = DdsConnectionState::DISCONNECTED;
        uint64_t session_generation = 0;
        uint64_t disconnect_epoch = 0;
        uint32_t connection_attempt = 0;
    };

    inline const char *dds_connection_state_name(DdsConnectionState state)
    {
        switch (state)
        {
        case DdsConnectionState::DISCONNECTED:
            return "DISCONNECTED";
        case DdsConnectionState::CONNECTING:
            return "CONNECTING";
        case DdsConnectionState::CONNECTED:
            return "CONNECTED";
        case DdsConnectionState::BACKOFF:
            return "BACKOFF";
        case DdsConnectionState::STOPPED:
            return "STOPPED";
        }
        return "UNKNOWN";
    }

    // Retry quickly at first, then cap retries so a long Agent outage does not
    // create a permanent busy loop or excessive log/network traffic.
    inline std::chrono::milliseconds dds_reconnect_delay(uint32_t attempt)
    {
        if (attempt <= 1)
            return std::chrono::seconds(1);
        if (attempt == 2)
            return std::chrono::seconds(2);
        if (attempt == 3)
            return std::chrono::seconds(4);
        return std::chrono::seconds(5);
    }

    // Control-thread view of the asynchronous DDS connection. The disconnect
    // epoch is deliberately independent of the latest state: if BACKOFF is
    // overwritten by a fast reconnect before the control thread reads it, the
    // epoch change still invalidates commands from the previous session.
    class DdsControlLinkState
    {
    public:
        bool observe(const DdsConnectionStatus &status)
        {
            const bool disconnect_observed =
                status.disconnect_epoch != disconnect_epoch_;
            state_ = status.state;
            session_generation_ = status.session_generation;
            disconnect_epoch_ = status.disconnect_epoch;
            return disconnect_observed;
        }

        bool accepts_setpoint(uint64_t session_generation) const
        {
            return state_ == DdsConnectionState::CONNECTED &&
                   session_generation != 0 &&
                   session_generation == session_generation_;
        }

        bool is_connected() const
        {
            return state_ == DdsConnectionState::CONNECTED;
        }

        uint64_t session_generation() const
        {
            return session_generation_;
        }

        uint64_t disconnect_epoch() const
        {
            return disconnect_epoch_;
        }

    private:
        DdsConnectionState state_ = DdsConnectionState::DISCONNECTED;
        uint64_t session_generation_ = 0;
        uint64_t disconnect_epoch_ = 0;
    };
} // namespace hydrox::sitl
