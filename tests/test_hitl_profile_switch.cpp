#include "gnc/control_factory.h"
#include "hydrox/runtime/hitl_profile_registry.h"
#include "hydrox/runtime/generated_fmuv6c_profiles.h"
#include "hydrox/runtime/hil_session_mapping.h"
#include "hydrox/runtime/hitl_supervisor.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

namespace
{
int expect(bool condition, const char *message)
{
    if (condition)
        return 0;
    std::fprintf(stderr, "FAIL: %s\n", message);
    return 1;
}

class FakeClock final : public hydrox::platform::Clock
{
public:
    hydrox::platform::MonotonicTimeUs now_us() const noexcept override
    {
        return now;
    }
    mutable hydrox::platform::MonotonicTimeUs now = 1'000'000;
};

class FakeSleeper final : public hydrox::platform::Sleeper
{
public:
    explicit FakeSleeper(FakeClock &clock) : clock_(clock) {}
    void sleep_for_us(hydrox::platform::MonotonicTimeUs us) noexcept override
    {
        clock_.now += us;
    }
    void sleep_until_us(
        hydrox::platform::MonotonicTimeUs deadline) noexcept override
    {
        if (clock_.now < deadline)
            clock_.now = deadline;
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
    explicit FakeStream(std::vector<uint8_t> inbound = {})
        : inbound_(std::move(inbound))
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
        uint8_t *data, std::size_t capacity) noexcept override
    {
        if (!inbound_delivered_ && !inbound_.empty())
        {
            if (inbound_.size() > capacity)
                return {0, hydrox::platform::IoStatus::Error};
            std::memcpy(data, inbound_.data(), inbound_.size());
            inbound_delivered_ = true;
            return {inbound_.size(), hydrox::platform::IoStatus::Ok};
        }
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
    std::vector<uint8_t> wire;

private:
    std::vector<uint8_t> inbound_;
    bool inbound_delivered_ = false;
};

class FakeBoard final : public hydrox::runtime::HitlBoard
{
public:
    FakeBoard(std::vector<uint8_t> inbound,
              std::string accepted_id = {},
              uint64_t accepted_fingerprint = 0)
        : sleeper_impl(clock_impl),
          stream_impl(std::move(inbound)),
          accepted_id_(std::move(accepted_id)),
          accepted_fingerprint_(accepted_fingerprint)
    {
    }

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
    bool physical_actuators_inhibited() const noexcept override { return true; }
    bool bootloader_reboot_supported() const noexcept override
    {
        return bootloader_supported;
    }
    bool load_vehicle_profile(
        hydrox::runtime::HitlVehicleProfile &) noexcept override
    {
        return false;
    }
    bool request_vehicle_profile(
        const char *profile_id,
        uint64_t fingerprint,
        uint32_t nonce) noexcept override
    {
        ++profile_requests;
        requested_id = profile_id != nullptr ? profile_id : "";
        requested_fingerprint = fingerprint;
        requested_nonce = nonce;
        const bool accepted = requested_id == accepted_id_ &&
                              fingerprint == accepted_fingerprint_;
        if (accepted)
        {
            command_connected = false;
            ++command_generation;
        }
        return accepted;
    }
    hydrox::runtime::HitlSessionRequestResult request_session_config(
        uint64_t profile_fingerprint,
        const hydrox::runtime::HilSessionConfigV1 &config,
        const hydrox::runtime::HilSessionDigest &digest,
        uint32_t nonce) noexcept override
    {
        ++session_requests;
        requested_session_profile = profile_fingerprint;
        requested_session = config;
        requested_session_digest = digest;
        requested_session_nonce = nonce;
        if (!accept_session)
            return {
                hydrox::runtime::HilSessionRejectReason::UnsupportedPolicy,
                hydrox::runtime::HilSessionField::FeedbackSource};
        command_connected = false;
        ++command_generation;
        return {
            hydrox::runtime::HilSessionRejectReason::None,
            hydrox::runtime::HilSessionField::None};
    }
    bool command_link_connected() const noexcept override
    {
        return command_connected;
    }
    uint64_t command_link_generation() const noexcept override
    {
        return command_generation;
    }
    bool poll_setpoint(hydrox::runtime::HitlSetpointSample &) noexcept override
    {
        return false;
    }
    bool should_exit() const noexcept override
    {
        return stream_impl.exit_requested;
    }

    FakeClock clock_impl;
    FakeSleeper sleeper_impl;
    FakeStream stream_impl;
    FakeWatchdog watchdog_impl;
    int profile_requests = 0;
    std::string requested_id;
    uint64_t requested_fingerprint = 0;
    uint32_t requested_nonce = 0;
    bool command_connected = true;
    uint64_t command_generation = 1;
    int session_requests = 0;
    uint64_t requested_session_profile = 0;
    hydrox::runtime::HilSessionConfigV1 requested_session{};
    hydrox::runtime::HilSessionDigest requested_session_digest{};
    uint32_t requested_session_nonce = 0;
    bool accept_session = false;
    bool bootloader_supported = false;

private:
    std::string accepted_id_;
    uint64_t accepted_fingerprint_ = 0;
};

std::vector<uint8_t> selection_frame(
    const char *profile_id,
    uint64_t fingerprint,
    uint32_t nonce,
    uint8_t mav_type)
{
    hydrox::HilProfileMsg message;
    message.fingerprint = fingerprint;
    message.nonce = nonce;
    message.operation = hydrox::HilProfileOperation::Select;
    message.mav_type = mav_type;
    const std::size_t length = std::strlen(profile_id);
    std::memcpy(message.profile_id.data(), profile_id, length + 1);
    message.valid = true;
    hydrox::MavlinkPacket packet;
    hydrox::MavlinkHIL codec(50, 191);
    if (!codec.encode_hil_profile(packet, message))
        return {};
    return {packet.data(), packet.data() + packet.size()};
}

std::vector<uint8_t> session_frame(
    uint64_t profile_fingerprint,
    uint32_t nonce,
    const hydrox::runtime::HilSessionConfigV1 &config)
{
    hydrox::HilSessionConfigMsg message;
    message.profile_fingerprint = profile_fingerprint;
    message.nonce = nonce;
    message.config = config;
    message.digest = hydrox::runtime::hil_session_digest(
        profile_fingerprint, config);
    message.digest_valid = true;
    message.valid = true;
    hydrox::MavlinkPacket packet;
    hydrox::MavlinkHIL codec(50, 191);
    if (!codec.encode_hil_session_config(packet, message))
        return {};
    return {packet.data(), packet.data() + packet.size()};
}

std::vector<uint8_t> bytes_from_hex(const char *text)
{
    const auto nibble = [](char value) -> uint8_t
    {
        if (value >= '0' && value <= '9')
            return static_cast<uint8_t>(value - '0');
        if (value >= 'a' && value <= 'f')
            return static_cast<uint8_t>(value - 'a' + 10);
        if (value >= 'A' && value <= 'F')
            return static_cast<uint8_t>(value - 'A' + 10);
        return 0xFF;
    };
    std::vector<uint8_t> result;
    if (text == nullptr)
        return result;
    const std::size_t length = std::strlen(text);
    if ((length % 2) != 0)
        return result;
    result.reserve(length / 2);
    for (std::size_t index = 0; index < length; index += 2)
    {
        const uint8_t high = nibble(text[index]);
        const uint8_t low = nibble(text[index + 1]);
        if (high > 0x0F || low > 0x0F)
            return {};
        result.push_back(static_cast<uint8_t>((high << 4) | low));
    }
    return result;
}

std::vector<uint8_t> uploader_reboot_frame(bool targeted)
{
    // Exact MAVLink 1 packets from PX4's official px4_uploader.py.
    return bytes_from_hex(targeted
        ? "fe2172ff004c00004040000000000000000000000000"
          "000000000000000000000000f600010000536b"
        : "fe2145ff004c00004040000000000000000000000000"
          "000000000000000000000000f600000000cc37");
}

std::vector<uint8_t> uploader_reboot_pair()
{
    std::vector<uint8_t> result = uploader_reboot_frame(false);
    const std::vector<uint8_t> targeted = uploader_reboot_frame(true);
    result.insert(result.end(), targeted.begin(), targeted.end());
    return result;
}

bool find_profile_status(
    const std::vector<uint8_t> &wire,
    hydrox::HilProfileOperation operation,
    const char *profile_id,
    uint64_t fingerprint,
    uint32_t nonce)
{
    hydrox::MavlinkHIL decoder;
    const auto frames = decoder.feed(wire.data(), wire.size());
    for (const auto &frame : frames)
    {
        if (frame.msg_id != hydrox::MSGID_HIL_PROFILE)
            continue;
        const auto status = decoder.parse_hil_profile(frame);
        if (status.valid && status.operation == operation &&
            status.fingerprint == fingerprint && status.nonce == nonce &&
            std::strcmp(status.profile_id.data(), profile_id) == 0)
            return true;
    }
    return false;
}

bool find_session_status(
    const std::vector<uint8_t> &wire,
    hydrox::HilSessionStatusOperation operation,
    uint64_t fingerprint,
    uint32_t nonce,
    const hydrox::runtime::HilSessionDigest &digest)
{
    hydrox::MavlinkHIL decoder;
    const auto frames = decoder.feed(wire.data(), wire.size());
    for (const auto &frame : frames)
    {
        if (frame.msg_id != hydrox::MSGID_HIL_SESSION_STATUS)
            continue;
        const auto status = decoder.parse_hil_session_status(frame);
        if (status.valid && status.operation == operation &&
            status.profile_fingerprint == fingerprint &&
            status.nonce == nonce &&
            hydrox::runtime::hil_session_digest_equal(
                status.digest, digest))
            return true;
    }
    return false;
}

hydrox::runtime::HitlVehicleProfile load_profile(
    const char *profile_id, uint64_t fingerprint, uint32_t nonce = 0)
{
    hydrox::runtime::HitlVehicleProfile profile;
    std::string error;
    if (!hydrox::runtime::load_compiled_hitl_profile(
            profile_id, fingerprint, nonce, profile, &error))
    {
        std::fprintf(stderr, "FAIL: cannot load %s: %s\n",
                     profile_id, error.c_str());
    }
    return profile;
}

hydrox::runtime::HitlSupervisor supervisor_for(
    FakeBoard &board,
    hydrox::runtime::HitlVehicleProfile profile)
{
    profile.runtime.motor = profile.control.motor;
    return hydrox::runtime::HitlSupervisor(
        board,
        profile,
        hydrox::build_control_stack(profile.control));
}
} // namespace

int main()
{
    int failures = 0;
    constexpr char kActiveId[] = "generic-auv-fin";
    constexpr uint64_t kActiveFingerprint = hydrox::runtime::kGeneratedFmuv6cProfiles[0].fingerprint;
    constexpr char kNextId[] = "X500";
    constexpr uint64_t kNextFingerprint = hydrox::runtime::kGeneratedFmuv6cProfiles[6].fingerprint;
    constexpr uint32_t kNonce = 0x10203040U;

    {
        const std::vector<uint8_t> bytes = uploader_reboot_pair();
        hydrox::MavlinkHIL codec;
        const auto frames = codec.feed(bytes.data(), bytes.size());
        failures += expect(
            frames.size() == 2 && frames[0].mavlink_version == 1 &&
                frames[1].mavlink_version == 1,
            "MAVLink deframer accepts both official uploader v1 packets");
        if (frames.size() == 2)
        {
            const auto broadcast = codec.parse_command_long(frames[0]);
            const auto targeted = codec.parse_command_long(frames[1]);
            failures += expect(
                broadcast.valid && targeted.valid &&
                    broadcast.command ==
                        hydrox::MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN &&
                    broadcast.params[0] == 3.0F &&
                    broadcast.target_system == 0 &&
                    targeted.target_system == 1,
                "official uploader packets decode as the strict reboot pair");
        }
    }

    {
        auto profile = load_profile(kActiveId, kActiveFingerprint);
        FakeBoard board(
            selection_frame(kActiveId, kActiveFingerprint, kNonce, 12));
        auto supervisor = supervisor_for(board, std::move(profile));
        failures += expect(supervisor.run() == 0,
                           "active profile handshake completes without restart");
        failures += expect(board.profile_requests == 0,
                           "active profile does not ask board to switch");
        failures += expect(
            find_profile_status(board.stream_impl.wire,
                                hydrox::HilProfileOperation::Ready,
                                kActiveId, kActiveFingerprint, kNonce),
            "active exact identity receives READY with the request nonce");
    }

    {
        auto profile = load_profile(kActiveId, kActiveFingerprint);
        constexpr char kUnknownId[] = "not-in-firmware";
        constexpr uint64_t kUnknownFingerprint = 0x0123456789ABCDEFULL;
        FakeBoard board(
            selection_frame(kUnknownId, kUnknownFingerprint, kNonce, 12));
        auto supervisor = supervisor_for(board, std::move(profile));
        failures += expect(supervisor.run() == 0,
                           "rejected profile leaves active supervisor running");
        failures += expect(board.profile_requests == 1,
                           "unknown exact identity is checked once");
        failures += expect(
            find_profile_status(board.stream_impl.wire,
                                hydrox::HilProfileOperation::Rejected,
                                kUnknownId, kUnknownFingerprint, kNonce),
            "unknown identity receives REJECTED with no fallback");
    }

    {
        auto profile = load_profile(kActiveId, kActiveFingerprint);
        FakeBoard board(
            selection_frame(kNextId, kNextFingerprint, kNonce, 2),
            kNextId,
            kNextFingerprint);
        auto supervisor = supervisor_for(board, std::move(profile));
        failures += expect(
            supervisor.run() == hydrox::runtime::kHitlProfileSwitchRequested,
            "accepted different profile exits the old state epoch");
        failures += expect(
            board.profile_requests == 1 &&
                board.requested_id == kNextId &&
                board.requested_fingerprint == kNextFingerprint &&
                board.requested_nonce == kNonce,
            "board stages only the exact requested identity and nonce");
        failures += expect(!board.command_connected &&
                               board.command_generation == 2,
                           "profile switch revokes command-link authority");
        const auto &channels =
            supervisor.flight_runtime().last_tick().actuator.ch;
        failures += expect(
            std::all_of(channels.begin(), channels.end(),
                        [](float value) { return value == 0.0F; }),
            "old runtime is neutral before it is destroyed");
        failures += expect(
            !find_profile_status(board.stream_impl.wire,
                                 hydrox::HilProfileOperation::Ready,
                                 kNextId, kNextFingerprint, kNonce),
            "old control stack never claims the new profile is READY");
    }

    {
        auto profile = load_profile(kNextId, kNextFingerprint, kNonce);
        FakeBoard board({});
        auto supervisor = supervisor_for(board, std::move(profile));
        failures += expect(supervisor.run() == 0,
                           "rebuilt profile supervisor starts normally");
        failures += expect(
            find_profile_status(board.stream_impl.wire,
                                hydrox::HilProfileOperation::Ready,
                                kNextId, kNextFingerprint, kNonce),
            "rebuilt stack emits READY with the staged nonce");
    }

    {
        auto profile = load_profile(kActiveId, kActiveFingerprint);
        FakeBoard board(uploader_reboot_pair());
        board.command_connected = false;
        board.bootloader_supported = true;
        auto supervisor = supervisor_for(board, std::move(profile));
        failures += expect(
            supervisor.run() ==
                hydrox::runtime::kHitlBootloaderRestartRequested,
            "official uploader pair exits the pre-Session maintenance epoch");
        const auto &channels =
            supervisor.flight_runtime().last_tick().actuator.ch;
        failures += expect(
            std::all_of(channels.begin(), channels.end(),
                        [](float value) { return value == 0.0F; }),
            "bootloader handoff leaves the runtime actuator state neutral");
    }

    {
        auto profile = load_profile(kActiveId, kActiveFingerprint);
        FakeBoard board(uploader_reboot_frame(true));
        board.command_connected = false;
        board.bootloader_supported = true;
        auto supervisor = supervisor_for(board, std::move(profile));
        failures += expect(
            supervisor.run() == 0,
            "a lone targeted reboot packet cannot enter the bootloader");
    }

    {
        auto profile = load_profile(kActiveId, kActiveFingerprint);
        FakeBoard board(uploader_reboot_pair());
        board.command_connected = true;
        board.bootloader_supported = true;
        auto supervisor = supervisor_for(board, std::move(profile));
        failures += expect(
            supervisor.run() == 0,
            "an active command link closes the bootloader maintenance window");
    }

    hydrox::runtime::HilSessionConfigV1 requested_session;
    requested_session.initial_down_mm = 12'500;
    requested_session.gps_origin_lat_e7 = 223'000'000;
    requested_session.gps_origin_lon_e7 = 1'141'700'000;
    const auto requested_digest =
        hydrox::runtime::hil_session_digest(
            kActiveFingerprint, requested_session);

    {
        auto profile = load_profile(kActiveId, kActiveFingerprint);
        profile.session_config = requested_session;
        profile.session_digest = requested_digest;
        profile.session_nonce = kNonce;
        profile.session_configured = true;
        hydrox::runtime::apply_hil_session_config(
            profile.session_config, profile.runtime, profile.sensors);
        FakeBoard board(uploader_reboot_pair());
        board.command_connected = false;
        board.bootloader_supported = true;
        auto supervisor = supervisor_for(board, std::move(profile));
        failures += expect(
            supervisor.run() == 0,
            "an active Session closes the bootloader maintenance window");
    }

    {
        auto profile = load_profile(kActiveId, kActiveFingerprint);
        FakeBoard board(
            session_frame(
                kActiveFingerprint, kNonce, requested_session));
        board.accept_session = true;
        auto supervisor = supervisor_for(board, std::move(profile));
        failures += expect(
            supervisor.run() ==
                hydrox::runtime::kHitlSessionConfigRestartRequested,
            "accepted Session Config exits the old runtime epoch");
        failures += expect(
            board.session_requests == 1 &&
                board.requested_session_profile == kActiveFingerprint &&
                board.requested_session_nonce == kNonce &&
                hydrox::runtime::hil_session_digest_equal(
                    board.requested_session_digest, requested_digest),
            "board stages the exact profile-bound Session identity");
        failures += expect(
            !find_session_status(
                board.stream_impl.wire,
                hydrox::HilSessionStatusOperation::Ready,
                kActiveFingerprint,
                kNonce,
                requested_digest),
            "old runtime never claims staged Session Config is READY");
    }

    {
        auto profile = load_profile(kActiveId, kActiveFingerprint);
        profile.session_config = requested_session;
        profile.session_digest = requested_digest;
        profile.session_nonce = kNonce;
        profile.session_configured = true;
        hydrox::runtime::apply_hil_session_config(
            profile.session_config, profile.runtime, profile.sensors);
        FakeBoard board({});
        auto supervisor = supervisor_for(board, std::move(profile));
        failures += expect(
            supervisor.run() == 0,
            "rebuilt Session supervisor starts normally");
        failures += expect(
            find_session_status(
                board.stream_impl.wire,
                hydrox::HilSessionStatusOperation::Ready,
                kActiveFingerprint,
                kNonce,
                requested_digest),
            "rebuilt runtime emits Session READY with digest and nonce");
    }

    {
        auto profile = load_profile(kActiveId, kActiveFingerprint);
        profile.session_config = requested_session;
        profile.session_digest = requested_digest;
        profile.session_nonce = 1;
        profile.session_configured = true;
        hydrox::runtime::apply_hil_session_config(
            profile.session_config, profile.runtime, profile.sensors);
        constexpr uint32_t kReconnectNonce = 0x50607080U;
        FakeBoard board(
            session_frame(
                kActiveFingerprint, kReconnectNonce, requested_session));
        auto supervisor = supervisor_for(board, std::move(profile));
        failures += expect(
            supervisor.run() == 0 && board.session_requests == 0,
            "identical active Session is idempotent and does not rebuild");
        failures += expect(
            find_session_status(
                board.stream_impl.wire,
                hydrox::HilSessionStatusOperation::Ready,
                kActiveFingerprint,
                kReconnectNonce,
                requested_digest),
            "identical active Session acknowledges the reconnect nonce");
    }

    if (failures == 0)
        std::printf("test_hitl_profile_switch: all checks passed\n");
    return failures == 0 ? 0 : 1;
}
