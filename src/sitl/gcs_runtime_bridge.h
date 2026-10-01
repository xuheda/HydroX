#pragma once

#include "hydrox/gcs/mavlink_protocol.h"
#include "hydrox/runtime/hil_runtime.h"
#include "mavlink_hil.h"
#include "sitl/sitl_platform.h"

#include <cstdint>

namespace hydrox::sitl
{
    class GcsRuntimeBridge
    {
    public:
        GcsRuntimeBridge(UdpSender &transport,
                         uint8_t system_id,
                         uint8_t component_id,
                         uint8_t mav_type);

        void set_link_connected(bool connected);
        void update_telemetry(const runtime::HilRuntimeTick &tick,
                              bool gps_valid,
                              const HilGpsMsg &gps);
        void service(runtime::HilRuntime &runtime, uint64_t now_us);
        bool send_statustext(uint8_t severity, const char *text);

    private:
        static bool send_packet(void *context,
                                const uint8_t *data,
                                std::size_t size);
        static void handle_command(void *context,
                                   const gcs::Command &command);
        void apply_command(const gcs::Command &command);

        UdpSender &transport_;
        gcs::MavlinkProtocol protocol_;
        gcs::TelemetryState telemetry_{};
        runtime::HilRuntime *active_runtime_ = nullptr;
        uint64_t active_now_us_ = 0;
    };
} // namespace hydrox::sitl
