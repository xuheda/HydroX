#pragma once

#include "fossen_vehicle_params.h"
#include "hydrox/platform/byte_stream.h"
#include "hydrox/platform/clock.h"
#include "hydrox/platform/sleeper.h"
#include "hydrox/platform/watchdog.h"
#include "hydrox/runtime/hil_runtime.h"
#include "sensor_adapter.h"

#include <cstdint>
#include <array>

namespace hydrox::runtime
{
    struct HitlSetpointSample
    {
        GNCSetpoint setpoint{};
        GNCMode mode = GNCMode::DISABLED;
        platform::MonotonicTimeUs received_at_us = 0;
        uint64_t command_link_generation = 0;
    };

    enum class HitlHealthEvent : uint8_t
    {
        STARTING = 0,
        HIL_CONNECTED,
        HIL_DISCONNECTED,
        SENSOR_READY,
        COMMAND_ACCEPTED,
        COMMAND_REJECTED,
        FAILSAFE,
        STREAM_BACKPRESSURE,
        CONFIGURATION_ERROR,
    };

    struct HitlVehicleProfile
    {
        std::array<char, 64> profile_id{};
        /** Must equal VehicleBundle::fingerprint used to generate this profile. */
        uint64_t bundle_fingerprint = 0;
        FossenControlParams control{};
        HilRuntimeConfig runtime{};
        SensorAdapter::Params sensors{};
        uint8_t mav_type = 12;
    };

    /** Validate compiled profile safety bounds before constructing GNC. */
    bool validate_hitl_vehicle_profile(
        const HitlVehicleProfile &profile,
        const char *&error) noexcept;

    /** Board-owned services required by the platform-neutral HITL supervisor. */
    class HitlBoard
    {
    public:
        virtual ~HitlBoard() = default;

        virtual platform::Clock &clock() noexcept = 0;
        virtual platform::Sleeper &sleeper() noexcept = 0;
        virtual platform::ByteStream &hil_stream() noexcept = 0;
        virtual platform::Watchdog &watchdog() noexcept = 0;

        virtual bool physical_actuators_inhibited() const noexcept = 0;
        virtual bool load_vehicle_profile(
            HitlVehicleProfile &profile) noexcept = 0;

        virtual bool command_link_connected() const noexcept = 0;
        virtual uint64_t command_link_generation() const noexcept = 0;
        virtual bool poll_setpoint(HitlSetpointSample &sample) noexcept = 0;

        virtual bool should_exit() const noexcept = 0;
        virtual void notify(HitlHealthEvent, const char *) noexcept {}
    };
} // namespace hydrox::runtime

/** Implemented by the real Pixhawk 6C BSP, never by application code. */
extern "C" hydrox::runtime::HitlBoard *hydrox_hitl_board() noexcept;
