#include "hydrox/runtime/hitl_board.h"

#include <cstdio>

extern "C" int hydrox_hitl_main(int argc, char *argv[]);

namespace
{
    class NullClock final : public hydrox::platform::Clock
    {
    public:
        hydrox::platform::MonotonicTimeUs now_us() const noexcept override
        {
            return 0;
        }
    };

    class NullSleeper final : public hydrox::platform::Sleeper
    {
    public:
        void sleep_for_us(hydrox::platform::MonotonicTimeUs) noexcept override {}
        void sleep_until_us(hydrox::platform::MonotonicTimeUs) noexcept override {}
    };

    class NullStream final : public hydrox::platform::ByteStream
    {
    public:
        bool open() noexcept override { return false; }
        void close() noexcept override {}
        bool is_open() const noexcept override { return false; }
        hydrox::platform::IoResult read(uint8_t *, std::size_t) noexcept override
        {
            return {0, hydrox::platform::IoStatus::Closed};
        }
        hydrox::platform::IoResult write(
            const uint8_t *, std::size_t) noexcept override
        {
            return {0, hydrox::platform::IoStatus::Closed};
        }
    };

    class NullWatchdog final : public hydrox::platform::Watchdog
    {
    public:
        bool start(uint32_t) noexcept override { return false; }
        void kick() noexcept override {}
        bool is_running() const noexcept override { return false; }
    };

    class InvalidProfileBoard final : public hydrox::runtime::HitlBoard
    {
    public:
        hydrox::platform::Clock &clock() noexcept override { return clock_; }
        hydrox::platform::Sleeper &sleeper() noexcept override { return sleeper_; }
        hydrox::platform::ByteStream &hil_stream() noexcept override { return stream_; }
        hydrox::platform::Watchdog &watchdog() noexcept override { return watchdog_; }
        bool physical_actuators_inhibited() const noexcept override { return true; }
        bool load_vehicle_profile(
            hydrox::runtime::HitlVehicleProfile &) noexcept override
        {
            return false;
        }
        bool command_link_connected() const noexcept override { return false; }
        uint64_t command_link_generation() const noexcept override { return 0; }
        bool poll_setpoint(
            hydrox::runtime::HitlSetpointSample &) noexcept override
        {
            return false;
        }
        bool should_exit() const noexcept override { return true; }

    private:
        NullClock clock_;
        NullSleeper sleeper_;
        NullStream stream_;
        NullWatchdog watchdog_;
    };

    InvalidProfileBoard board;
}

extern "C" hydrox::runtime::HitlBoard *hydrox_hitl_board() noexcept
{
    return &board;
}

int main()
{
    if (hydrox_hitl_main(0, nullptr) != 11)
    {
        std::fprintf(stderr,
                     "FAIL: HITL entry did not reject an invalid vehicle profile\n");
        return 1;
    }
    std::puts("PASS: Pixhawk HITL entry is linked and validates configuration");
    return 0;
}
