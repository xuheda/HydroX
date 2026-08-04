#include "hydrox/runtime/hitl_supervisor.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>

namespace
{
    int fail(const char *message)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        return 1;
    }

    uint16_t crc16(const uint8_t *data, std::size_t length,
                   uint16_t crc = 0xFFFF)
    {
        for (std::size_t i = 0; i < length; ++i)
        {
            uint8_t tmp = data[i] ^ static_cast<uint8_t>(crc & 0xFF);
            tmp ^= static_cast<uint8_t>((tmp << 4) & 0xFF);
            crc = static_cast<uint16_t>(
                (crc >> 8) ^ (static_cast<uint16_t>(tmp) << 8) ^
                (static_cast<uint16_t>(tmp) << 3) ^
                (static_cast<uint16_t>(tmp) >> 4));
        }
        return crc;
    }

    std::vector<uint8_t> hil_sensor_frame(uint64_t timestamp_us,
                                          uint8_t sequence)
    {
        std::array<uint8_t, 64> payload{};
        std::memcpy(payload.data(), &timestamp_us, sizeof(timestamp_us));

        std::vector<uint8_t> frame(10 + payload.size() + 2, 0);
        frame[0] = 0xFD;
        frame[1] = static_cast<uint8_t>(payload.size());
        frame[4] = sequence;
        frame[5] = 42;
        frame[6] = 1;
        frame[7] = static_cast<uint8_t>(hydrox::MSGID_HIL_SENSOR & 0xFF);
        std::memcpy(frame.data() + 10, payload.data(), payload.size());
        uint16_t crc = crc16(frame.data() + 1, 9 + payload.size());
        constexpr uint8_t crc_extra = 108;
        crc = crc16(&crc_extra, 1, crc);
        frame[10 + payload.size()] = static_cast<uint8_t>(crc & 0xFF);
        frame[11 + payload.size()] = static_cast<uint8_t>(crc >> 8);
        return frame;
    }

    class FakeClock final : public hydrox::platform::Clock
    {
    public:
        hydrox::platform::MonotonicTimeUs now_us() const noexcept override
        {
            return now;
        }
        mutable hydrox::platform::MonotonicTimeUs now = 10'000'000;
    };

    class FakeSleeper final : public hydrox::platform::Sleeper
    {
    public:
        explicit FakeSleeper(FakeClock &clock) : clock_(clock) {}
        void sleep_for_us(
            hydrox::platform::MonotonicTimeUs duration_us) noexcept override
        {
            clock_.now += duration_us;
        }
        void sleep_until_us(
            hydrox::platform::MonotonicTimeUs deadline_us) noexcept override
        {
            if (clock_.now < deadline_us)
                clock_.now = deadline_us;
        }

    private:
        FakeClock &clock_;
    };

    class FakeWatchdog final : public hydrox::platform::Watchdog
    {
    public:
        bool start(uint32_t) noexcept override
        {
            running = true;
            return true;
        }
        void kick() noexcept override { ++kicks; }
        bool is_running() const noexcept override { return running; }
        bool running = false;
        int kicks = 0;
    };

    class FakeStream final : public hydrox::platform::ByteStream
    {
    public:
        explicit FakeStream(FakeClock &clock)
            : clock_(clock),
              first_(hil_sensor_frame(1'000'000, 1)),
              second_(hil_sensor_frame(1'010'000, 2))
        {
        }

        bool open() noexcept override
        {
            opened = true;
            return true;
        }
        void close() noexcept override { opened = false; }
        bool is_open() const noexcept override { return opened; }

        hydrox::platform::IoResult read(
            uint8_t *buffer, std::size_t capacity) noexcept override
        {
            ++read_calls;
            clock_.now += 10'000;
            if (read_calls == 1)
                return copy(first_, buffer, capacity);
            if (read_calls == 3)
                return copy(second_, buffer, capacity);
            if (read_calls >= 4)
                exit_requested = true;
            return {0, hydrox::platform::IoStatus::WouldBlock};
        }

        hydrox::platform::IoResult write(
            const uint8_t *data, std::size_t size) noexcept override
        {
            wire.insert(wire.end(), data, data + size);
            return {size, hydrox::platform::IoStatus::Ok};
        }

        bool opened = false;
        bool exit_requested = false;
        int read_calls = 0;
        std::vector<uint8_t> wire;

    private:
        static hydrox::platform::IoResult copy(
            const std::vector<uint8_t> &source,
            uint8_t *destination,
            std::size_t capacity)
        {
            if (source.size() > capacity)
                return {0, hydrox::platform::IoStatus::Error};
            std::memcpy(destination, source.data(), source.size());
            return {source.size(), hydrox::platform::IoStatus::Ok};
        }

        FakeClock &clock_;
        std::vector<uint8_t> first_;
        std::vector<uint8_t> second_;
    };

    class FakeBoard final : public hydrox::runtime::HitlBoard
    {
    public:
        FakeBoard() : sleeper_impl(clock_impl), stream_impl(clock_impl) {}

        hydrox::platform::Clock &clock() noexcept override { return clock_impl; }
        hydrox::platform::Sleeper &sleeper() noexcept override
        {
            return sleeper_impl;
        }
        hydrox::platform::ByteStream &hil_stream() noexcept override
        {
            return stream_impl;
        }
        hydrox::platform::Watchdog &watchdog() noexcept override
        {
            return watchdog_impl;
        }
        bool physical_actuators_inhibited() const noexcept override
        {
            return actuators_inhibited;
        }
        bool load_vehicle_profile(
            hydrox::runtime::HitlVehicleProfile &) noexcept override
        {
            return false;
        }
        bool command_link_connected() const noexcept override { return true; }
        uint64_t command_link_generation() const noexcept override { return 1; }
        bool poll_setpoint(
            hydrox::runtime::HitlSetpointSample &sample) noexcept override
        {
            if (command_delivered || stream_impl.read_calls < 1)
                return false;
            command_delivered = true;
            sample.mode = hydrox::GNCMode::DEPTH_HOLD;
            sample.setpoint.surge_ref = 1.0;
            sample.received_at_us = clock_impl.now;
            sample.command_link_generation = 1;
            return true;
        }
        bool should_exit() const noexcept override
        {
            return stream_impl.exit_requested;
        }
        void notify(hydrox::runtime::HitlHealthEvent event,
                    const char *) noexcept override
        {
            events.push_back(event);
        }

        bool actuators_inhibited = true;
        bool command_delivered = false;
        FakeClock clock_impl;
        FakeSleeper sleeper_impl;
        FakeStream stream_impl;
        FakeWatchdog watchdog_impl;
        std::vector<hydrox::runtime::HitlHealthEvent> events;
    };

    class ConstantController final : public hydrox::IController
    {
    public:
        void reset(const hydrox::NavigationState &) override {}
        void set_mode(hydrox::GNCMode mode) override { mode_ = mode; }
        void set_setpoint(const hydrox::GNCSetpoint &) override {}
        hydrox::Wrench update(const hydrox::NavigationState &, double) override
        {
            hydrox::Wrench wrench = hydrox::Wrench::Zero();
            wrench[0] = 10.0;
            return wrench;
        }

    private:
        hydrox::GNCMode mode_ = hydrox::GNCMode::DISABLED;
    };

    class ConstantAllocator final : public hydrox::IAllocator
    {
    public:
        hydrox::ActuatorCmd allocate(
            const hydrox::Wrench &wrench, double) const override
        {
            hydrox::ActuatorCmd command;
            command.ch[0] = static_cast<float>(wrench[0] / 20.0);
            return command;
        }
    };

    hydrox::ControlStack stack()
    {
        hydrox::ControlStack result;
        result.controller = std::make_unique<ConstantController>();
        result.allocator = std::make_unique<ConstantAllocator>();
        return result;
    }
}

int main()
{
    hydrox::runtime::HitlVehicleProfile profile;
    profile.profile_id[0] = 't';
    profile.bundle_fingerprint = 1;
    profile.control.valid = true;
    profile.control.allocator.D_prop = 0.14;
    profile.control.allocator.n_max_rpm = 1500.0;
    profile.control.allocator.max_thrust_N = 100.0;
    profile.runtime.nominal_dt_s = 0.01;
    profile.runtime.max_sensor_dt_s = 0.25;
    profile.runtime.setpoint_timeout_us = 2'000'000;
    profile.runtime.sensor_timeout_us = 500'000;

    const char *profile_error = nullptr;
    if (!hydrox::runtime::validate_hitl_vehicle_profile(
            profile, profile_error))
    {
        return fail(profile_error != nullptr
                        ? profile_error
                        : "valid embedded profile was rejected");
    }

    FakeBoard board;
    hydrox::runtime::HitlSupervisor supervisor(board, profile, stack());
    if (supervisor.run() != 0)
        return fail("supervisor did not complete fake board session");
    if (!board.watchdog_impl.running || board.watchdog_impl.kicks == 0)
        return fail("hardware watchdog was not started and serviced");
    if (!board.command_delivered)
        return fail("fresh companion command was not polled");

    hydrox::MavlinkHIL decoder;
    const std::vector<hydrox::MavFrame> frames = decoder.feed(
        board.stream_impl.wire.data(), board.stream_impl.wire.size());
    bool saw_safe = false;
    bool saw_armed = false;
    for (const hydrox::MavFrame &frame : frames)
    {
        if (frame.msg_id != hydrox::MSGID_HIL_ACTUATOR_CONTROLS ||
            frame.payload.size() < 81)
        {
            continue;
        }
        float channel_zero = 0.0f;
        std::memcpy(&channel_zero, frame.payload.data() + 8, sizeof(float));
        const uint8_t mode = frame.payload[80];
        const bool armed =
            (mode & hydrox::runtime::kMavModeFlagSafetyArmed) != 0;
        saw_safe = saw_safe || (!armed && std::abs(channel_zero) < 1e-6f);
        saw_armed = saw_armed || (armed && std::abs(channel_zero - 0.5f) < 1e-6f);
    }
    if (!saw_safe || !saw_armed)
        return fail("HITL wire did not contain both safe and armed actuator states");

    FakeBoard unsafe_board;
    unsafe_board.actuators_inhibited = false;
    hydrox::runtime::HitlSupervisor unsafe_supervisor(
        unsafe_board, profile, stack());
    if (unsafe_supervisor.run() != 20)
        return fail("supervisor started with physical actuators enabled");

    std::puts("PASS: Pixhawk HITL supervisor end-to-end contract");
    return 0;
}
