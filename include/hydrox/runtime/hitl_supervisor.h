#pragma once

#include "gnc/control_factory.h"
#include "hydrox/runtime/fixed_frame_sender.h"
#include "hydrox/runtime/hitl_board.h"
#include "mavlink_hil.h"

#include <cstdint>

namespace hydrox::runtime
{
    class HitlSupervisor
    {
    public:
        HitlSupervisor(HitlBoard &board,
                       const HitlVehicleProfile &profile,
                       ControlStack control_stack);

        int run();
        const HilRuntime &flight_runtime() const noexcept { return runtime_; }

    private:
        static void visit_frame(void *context, const MavFrame &frame);
        void on_frame(const MavFrame &frame);
        void service_command_link(platform::MonotonicTimeUs now_us);
        bool send_last_actuator(platform::MonotonicTimeUs now_us);
        bool send_heartbeat(platform::MonotonicTimeUs now_us);
        bool send_packet(const MavlinkPacket &packet,
                         platform::MonotonicTimeUs now_us);

        HitlBoard &board_;
        MavlinkHIL codec_;
        SensorAdapter sensor_adapter_;
        HilRuntime runtime_;
        FixedFrameSender sender_;
        uint8_t mav_type_ = 12;
        uint64_t last_sensor_time_us_ = 0;
        uint64_t command_generation_ = 0;
        platform::MonotonicTimeUs next_heartbeat_us_ = 0;
        bool command_connected_ = false;
        bool simulator_paused_ = false;
        bool stream_failed_ = false;
    };
} // namespace hydrox::runtime
