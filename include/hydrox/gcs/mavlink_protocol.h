#pragma once

#include "types.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <limits>

namespace hydrox::gcs
{
    enum class CommandKind : uint8_t
    {
        SET_ARMED = 0,
        SET_MODE,
        RESUME_CONTROL,
    };

    enum class CommandResult : uint8_t
    {
        ACCEPTED = 0,
        TEMPORARILY_REJECTED = 1,
        DENIED = 2,
        UNSUPPORTED = 3,
        FAILED = 4,
    };

    struct Command
    {
        CommandKind kind = CommandKind::SET_MODE;
        bool armed = false;
        GNCMode mode = GNCMode::DISABLED;
        uint16_t mav_command = 0;
        uint8_t source_system = 0;
        uint8_t source_component = 0;
        bool needs_ack = false;
    };

    struct TelemetryState
    {
        uint64_t monotonic_time_us = 0;
        uint32_t time_boot_ms = 0;
        uint8_t mav_type = 12;
        GNCMode mode = GNCMode::DISABLED;
        bool link_connected = false;
        bool armed = false;
        bool actuator_authorized = false;
        bool ekf_initialized = false;
        bool gps_valid = false;

        float roll_rad = 0.0f;
        float pitch_rad = 0.0f;
        float yaw_rad = 0.0f;
        float roll_rate_radps = 0.0f;
        float pitch_rate_radps = 0.0f;
        float yaw_rate_radps = 0.0f;

        float position_n_m = 0.0f;
        float position_e_m = 0.0f;
        float position_d_m = 0.0f;
        float velocity_n_mps = 0.0f;
        float velocity_e_mps = 0.0f;
        float velocity_d_mps = 0.0f;

        int32_t latitude_deg7 = 0;
        int32_t longitude_deg7 = 0;
        int32_t altitude_msl_mm = 0;
        float battery_remaining_pct = -1.0f;
        float ground_speed_mps = 0.0f;
        float equivalent_airspeed_mps = std::numeric_limits<float>::quiet_NaN();
        uint16_t throttle_pct = 0;
    };

    class MavlinkProtocol
    {
    public:
        using PacketSink = bool (*)(void *context,
                                    const uint8_t *data,
                                    std::size_t size);
        using CommandSink = void (*)(void *context, const Command &command);

        explicit MavlinkProtocol(uint8_t system_id = 1,
                                 uint8_t component_id = 1,
                                 uint8_t mav_type = 12);
        ~MavlinkProtocol();

        MavlinkProtocol(const MavlinkProtocol &) = delete;
        MavlinkProtocol &operator=(const MavlinkProtocol &) = delete;

        void ingest(const uint8_t *data,
                    std::size_t size,
                    uint64_t now_us,
                    void *command_context,
                    CommandSink command_sink,
                    void *packet_context,
                    PacketSink packet_sink);

        void update(const TelemetryState &state,
                    void *packet_context,
                    PacketSink packet_sink);

        bool acknowledge(const Command &command,
                         CommandResult result,
                         void *packet_context,
                         PacketSink packet_sink);

        bool send_statustext(uint8_t severity,
                             const char *text,
                             void *packet_context,
                             PacketSink packet_sink);

        uint64_t received_message_count() const noexcept;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };
} // namespace hydrox::gcs
