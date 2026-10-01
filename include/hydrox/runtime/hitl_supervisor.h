#pragma once

#include "gnc/control_factory.h"
#include "hydrox/runtime/fixed_frame_sender.h"
#include "hydrox/runtime/hil_session_driver.h"
#include "hydrox/runtime/hitl_board.h"
#include "mavlink_hil.h"

#include <array>
#include <cstdint>

namespace hydrox::runtime
{
    constexpr int kHitlProfileSwitchRequested = 30;
    constexpr int kHitlSessionConfigRestartRequested = 31;
    constexpr int kHitlBootloaderRestartRequested = 32;

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
        void handle_bootloader_reboot_command(
            const MavFrame &frame,
            platform::MonotonicTimeUs now_us);
        void service_command_link(platform::MonotonicTimeUs now_us);
        void notify_safety_status_transition();
        bool send_last_actuator(platform::MonotonicTimeUs now_us);
        bool send_heartbeat(platform::MonotonicTimeUs now_us);
        bool send_fc_state(platform::MonotonicTimeUs now_us);
        bool send_control_trace(platform::MonotonicTimeUs now_us);
        bool send_packet(const MavlinkPacket &packet,
                         platform::MonotonicTimeUs now_us);
        bool send_profile_status(
            HilProfileOperation operation,
            const char *profile_id,
            uint64_t fingerprint,
            uint32_t nonce,
            uint8_t mav_type,
            platform::MonotonicTimeUs now_us);
        bool send_session_status(
            HilSessionStatusOperation operation,
            const HilSessionDigest &digest,
            uint32_t nonce,
            HilSessionRejectReason reason,
            HilSessionField field,
            platform::MonotonicTimeUs now_us);

        HitlBoard &board_;
        MavlinkHIL codec_;
        HilRuntime runtime_;
        HilSessionDriver session_;
        FixedFrameSender sender_;
        std::array<char, 64> profile_id_{};
        uint64_t profile_fingerprint_ = 0;
        uint32_t selection_nonce_ = 0;
        uint8_t mav_type_ = 12;
        HilSessionConfigV1 session_config_{};
        HilSessionDigest session_digest_{};
        uint32_t session_nonce_ = 0;
        uint64_t command_generation_ = 0;
        platform::MonotonicTimeUs next_heartbeat_us_ = 0;
        platform::MonotonicTimeUs next_fc_state_us_ = 0;
        uint8_t actuator_channel_count_ = 0;
        uint64_t last_safety_transition_sequence_ = 0;
        bool command_connected_ = false;
        bool stream_failed_ = false;
        bool profile_switch_requested_ = false;
        bool session_configured_ = false;
        bool session_config_restart_requested_ = false;
        platform::MonotonicTimeUs bootloader_pair_started_us_ = 0;
        bool bootloader_broadcast_seen_ = false;
        bool bootloader_reboot_requested_ = false;
    };
} // namespace hydrox::runtime
