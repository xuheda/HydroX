#include "fmu_v6c.h"
#include "hydrox/platform/nuttx/nuttx_clock.h"
#include "hydrox/platform/nuttx/nuttx_serial_byte_stream.h"
#include "hydrox/platform/nuttx/nuttx_sleeper.h"
#include "hydrox/platform/nuttx/nuttx_watchdog.h"
#include "hydrox/runtime/hitl_board.h"
#include "hydrox/runtime/hitl_command_codec.h"
#include "hydrox/runtime/hitl_profile_registry.h"
#include "hydrox/runtime/hil_session_mapping.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <syslog.h>

namespace
{
using hydrox::platform::IoStatus;
using hydrox::platform::MonotonicTimeUs;
using hydrox::runtime::HitlCommandFrame;
using hydrox::runtime::HitlHealthEvent;
using hydrox::runtime::HitlSetpointSample;
using hydrox::runtime::HitlVehicleProfile;

constexpr const char *kHilDevice = "/dev/ttyS5";      // TELEM1 / UART7
constexpr const char *kCommandDevice = "/dev/ttyS3"; // TELEM2 / UART5
constexpr uint32_t kLinkBaud = 921600;
constexpr bool kHilHardwareFlowControl = false;
constexpr bool kCommandHardwareFlowControl = true;
constexpr MonotonicTimeUs kCommandTimeoutUs =
    hydrox::runtime::kDefaultHilSetpointTimeoutUs;
constexpr MonotonicTimeUs kOpenRetryUs = 250000;
constexpr char kDefaultProfileId[] = "generic-auv-fin";
constexpr uint64_t kDefaultProfileFingerprint = 0xDD7F5B0822D68E33ULL;
constexpr std::size_t kProfileIdCapacity = 64;

bool sequence_is_newer(uint16_t current, uint16_t previous) noexcept
{
    return static_cast<int16_t>(current - previous) > 0;
}

class FmuV6cHitlBoard final : public hydrox::runtime::HitlBoard
{
public:
    FmuV6cHitlBoard() noexcept
        : hil_stream_(kHilDevice, kLinkBaud, kHilHardwareFlowControl),
          command_stream_(kCommandDevice, kLinkBaud,
                          kCommandHardwareFlowControl)
    {
        std::memcpy(selected_profile_id_.data(),
                    kDefaultProfileId,
                    sizeof(kDefaultProfileId));
        selected_fingerprint_ = kDefaultProfileFingerprint;
    }

    hydrox::platform::Clock &clock() noexcept override { return clock_; }
    hydrox::platform::Sleeper &sleeper() noexcept override { return sleeper_; }
    hydrox::platform::ByteStream &hil_stream() noexcept override
    {
        return hil_stream_;
    }
    hydrox::platform::Watchdog &watchdog() noexcept override
    {
        return watchdog_;
    }

    bool physical_actuators_inhibited() const noexcept override
    {
        return fmu_v6c_outputs_are_inhibited();
    }

    bool bootloader_reboot_supported() const noexcept override
    {
        return true;
    }

    bool reboot_to_bootloader() noexcept override
    {
        disconnect_command_link();
        hil_stream_.close();
        fmu_v6c_reboot_to_bootloader();
        return false;
    }

    bool load_vehicle_profile(HitlVehicleProfile &profile) noexcept override
    {
        if (!hydrox::runtime::load_compiled_hitl_profile(
            selected_profile_id_.data(),
            selected_fingerprint_,
            selected_nonce_,
            profile,
            nullptr))
            return false;
        if (!session_configured_)
            return true;
        hydrox::runtime::apply_hil_session_config(
            selected_session_config_, profile.runtime, profile.sensors);
        profile.session_config = selected_session_config_;
        profile.session_digest = selected_session_digest_;
        profile.session_nonce = selected_session_nonce_;
        profile.session_configured = true;
        return true;
    }

    bool request_vehicle_profile(
        const char *profile_id,
        uint64_t fingerprint,
        uint32_t selection_nonce) noexcept override
    {
        if (profile_id == nullptr || profile_id[0] == '\0' ||
            fingerprint == 0 || selection_nonce == 0)
            return false;

        hydrox::runtime::HitlProfileIdentity identity;
        bool exact_match = false;
        for (std::size_t index = 0;
             hydrox::runtime::compiled_hitl_profile_identity(index, identity);
             ++index)
        {
            if (identity.fingerprint == fingerprint &&
                std::strcmp(identity.profile_id, profile_id) == 0)
            {
                exact_match = true;
                break;
            }
        }
        if (!exact_match)
            return false;

        const std::size_t length = std::strlen(profile_id);
        if (length == 0 || length >= kProfileIdCapacity)
            return false;
        disconnect_command_link();
        selected_profile_id_.fill('\0');
        std::memcpy(selected_profile_id_.data(), profile_id, length);
        selected_fingerprint_ = fingerprint;
        selected_nonce_ = selection_nonce;
        selected_session_config_ = {};
        selected_session_digest_.fill(0);
        selected_session_nonce_ = 0;
        session_configured_ = false;
        return true;
    }

    hydrox::runtime::HitlSessionRequestResult request_session_config(
        uint64_t profile_fingerprint,
        const hydrox::runtime::HilSessionConfigV1 &config,
        const hydrox::runtime::HilSessionDigest &digest,
        uint32_t nonce) noexcept override
    {
        using hydrox::runtime::HilSessionField;
        using hydrox::runtime::HilSessionRejectReason;
        if (profile_fingerprint != selected_fingerprint_)
            return {HilSessionRejectReason::ProfileMismatch,
                    HilSessionField::None};
        if (nonce == 0)
            return {HilSessionRejectReason::StaleNonce,
                    HilSessionField::None};
        if (!hydrox::runtime::hil_session_digest_equal(
                digest,
                hydrox::runtime::hil_session_digest(
                    profile_fingerprint, config)))
            return {HilSessionRejectReason::HashMismatch,
                    HilSessionField::None};

        HilSessionField field = HilSessionField::None;
        if (!hydrox::runtime::validate_hil_session_config(config, field))
            return {config.schema_version !=
                            hydrox::runtime::kHilSessionConfigVersion
                        ? HilSessionRejectReason::UnsupportedVersion
                        : HilSessionRejectReason::InvalidRange,
                    field};

        // Release HITL never accepts simulator truth as control authority.
        if (config.feedback_source !=
                hydrox::runtime::HilSessionFeedbackSource::EstimatedState)
            return {HilSessionRejectReason::UnsupportedPolicy,
                    HilSessionField::FeedbackSource};
        if (config.allow_truth_heading_aid())
            return {HilSessionRejectReason::UnsupportedPolicy,
                    HilSessionField::Flags};

        // Capability advertising is not present in Profile V1. Fail closed
        // instead of pretending a requested sensor set was checked.
        if (config.required_sensor_mask != 0)
            return {HilSessionRejectReason::MissingSensorCapability,
                    HilSessionField::RequiredSensors};

        disconnect_command_link();
        selected_session_config_ = config;
        selected_session_digest_ = digest;
        selected_session_nonce_ = nonce;
        session_configured_ = true;
        return {HilSessionRejectReason::None, HilSessionField::None};
    }

    bool command_link_connected() const noexcept override
    {
        service_command_link();
        return command_connected_;
    }

    uint64_t command_link_generation() const noexcept override
    {
        service_command_link();
        return command_generation_;
    }

    bool poll_setpoint(HitlSetpointSample &sample) noexcept override
    {
        service_command_link();
        if (!pending_command_ || !command_connected_)
            return false;
        sample = pending_sample_;
        pending_command_ = false;
        return true;
    }

    bool should_exit() const noexcept override { return false; }

    void notify(HitlHealthEvent event, const char *message) noexcept override
    {
        static constexpr const char *names[] =
        {
            "starting", "hil_connected", "hil_disconnected", "sensor_ready",
            "command_accepted", "command_rejected", "failsafe",
            "stream_backpressure", "configuration_error", "safety_transition",
            "bootloader_reboot_requested"
        };
        const unsigned int index = static_cast<unsigned int>(event);
        const char *name = index < sizeof(names) / sizeof(names[0])
                               ? names[index]
                               : "unknown";
        syslog(event == HitlHealthEvent::CONFIGURATION_ERROR ||
                       event == HitlHealthEvent::FAILSAFE
                   ? LOG_ERR : LOG_INFO,
               "HydroX HITL [%s] %s\n", name,
               message != nullptr ? message : "");
    }

private:
    void disconnect_command_link() const noexcept
    {
        command_stream_.close();
        decoder_.reset();
        pending_command_ = false;
        have_sequence_ = false;
        if (command_connected_)
        {
            command_connected_ = false;
            ++command_generation_;
        }
    }

    void accept_command(const HitlCommandFrame &frame,
                        MonotonicTimeUs now_us) const noexcept
    {
        if (have_sender_generation_ &&
            frame.sender_generation == sender_generation_ &&
            have_sequence_ && !sequence_is_newer(frame.sequence, last_sequence_))
            return;

        if (!command_connected_ || !have_sender_generation_ ||
            frame.sender_generation != sender_generation_)
        {
            ++command_generation_;
            have_sequence_ = false;
        }

        sender_generation_ = frame.sender_generation;
        have_sender_generation_ = true;
        last_sequence_ = frame.sequence;
        have_sequence_ = true;
        last_command_us_ = now_us;
        command_connected_ = true;
        pending_sample_ = frame.sample;
        pending_sample_.received_at_us = now_us;
        pending_sample_.command_link_generation = command_generation_;
        pending_command_ = true;
    }

    void service_command_link() const noexcept
    {
        const MonotonicTimeUs now_us = clock_.now_us();
        if (!command_stream_.is_open())
        {
            if (now_us < next_open_attempt_us_)
                return;
            next_open_attempt_us_ = now_us + kOpenRetryUs;
            if (!command_stream_.open())
                return;
        }

        std::array<uint8_t, 256> bytes{};
        for (int pass = 0; pass < 4; ++pass)
        {
            const auto result = command_stream_.read(bytes.data(), bytes.size());
            if (result.status == IoStatus::Ok && result.size > 0)
            {
                HitlCommandFrame frame{};
                if (decoder_.feed(bytes.data(), result.size, frame))
                    accept_command(frame, now_us);
                continue;
            }
            if (result.status == IoStatus::Error ||
                result.status == IoStatus::Closed)
            {
                disconnect_command_link();
                next_open_attempt_us_ = now_us + kOpenRetryUs;
            }
            break;
        }

        if (command_connected_ &&
            now_us - last_command_us_ > kCommandTimeoutUs)
            disconnect_command_link();
    }

    mutable hydrox::platform::nuttx::NuttxClock clock_{};
    hydrox::platform::nuttx::NuttxSleeper sleeper_;
    hydrox::platform::nuttx::NuttxSerialByteStream hil_stream_;
    mutable hydrox::platform::nuttx::NuttxSerialByteStream command_stream_;
    hydrox::platform::nuttx::NuttxWatchdog watchdog_{};
    mutable hydrox::runtime::HitlCommandDecoder decoder_{};
    mutable HitlSetpointSample pending_sample_{};
    mutable MonotonicTimeUs last_command_us_ = 0;
    mutable MonotonicTimeUs next_open_attempt_us_ = 0;
    mutable uint64_t command_generation_ = 0;
    mutable uint32_t sender_generation_ = 0;
    mutable uint16_t last_sequence_ = 0;
    std::array<char, kProfileIdCapacity> selected_profile_id_{};
    uint64_t selected_fingerprint_ = 0;
    uint32_t selected_nonce_ = 0;
    hydrox::runtime::HilSessionConfigV1 selected_session_config_{};
    hydrox::runtime::HilSessionDigest selected_session_digest_{};
    uint32_t selected_session_nonce_ = 0;
    mutable bool command_connected_ = false;
    mutable bool pending_command_ = false;
    mutable bool have_sender_generation_ = false;
    mutable bool have_sequence_ = false;
    bool session_configured_ = false;
};

FmuV6cHitlBoard g_board;
}

extern "C" hydrox::runtime::HitlBoard *hydrox_hitl_board() noexcept
{
    return &g_board;
}
