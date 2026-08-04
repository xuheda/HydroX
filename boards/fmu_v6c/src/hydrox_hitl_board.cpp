#include "fmu_v6c.h"
#include "hydrox/platform/nuttx/nuttx_clock.h"
#include "hydrox/platform/nuttx/nuttx_serial_byte_stream.h"
#include "hydrox/platform/nuttx/nuttx_sleeper.h"
#include "hydrox/platform/nuttx/nuttx_watchdog.h"
#include "hydrox/runtime/hitl_board.h"
#include "hydrox/runtime/hitl_command_codec.h"

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
constexpr MonotonicTimeUs kCommandTimeoutUs = 500000;
constexpr MonotonicTimeUs kOpenRetryUs = 250000;
constexpr uint64_t kBundleFingerprint = 0xDD7F5B0822D68E33ULL;

bool sequence_is_newer(uint16_t current, uint16_t previous) noexcept
{
    return static_cast<int16_t>(current - previous) > 0;
}

class FmuV6cHitlBoard final : public hydrox::runtime::HitlBoard
{
public:
    FmuV6cHitlBoard() noexcept
        : sleeper_(clock_),
          hil_stream_(kHilDevice, kLinkBaud, true),
          command_stream_(kCommandDevice, kLinkBaud, true)
    {
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

    bool load_vehicle_profile(HitlVehicleProfile &profile) noexcept override
    {
        profile = HitlVehicleProfile{};
        constexpr char profile_id[] = "generic-auv-fin@DD7F5B0822D68E33";
        std::memcpy(profile.profile_id.data(), profile_id, sizeof(profile_id));
        profile.bundle_fingerprint = kBundleFingerprint;
        profile.mav_type = 12; // MAV_TYPE_SUBMARINE

        auto &control = profile.control;
        control.valid = true;
        control.loaded_from_json = false;
        control.archetype = hydrox::VehicleArchetype::SlenderBodyFin;
        control.vehicle_class = hydrox::VehicleClass::UUV;
        control.vehicle_type = "generic-auv-fin";
        control.source_path = "compiled:profiles/generic-auv-fin/vehicle-bundle.json";
        control.mass_total = 70.0;
        control.M44_pitch = 26.0;

        control.allocator.rho = 1028.0;
        control.allocator.S_fin = 0.04;
        control.allocator.CL_s = 4.0;
        control.allocator.CL_r = 4.0;
        control.allocator.x_fin = 0.70;
        control.allocator.delta_max_deg = 25.0;
        control.allocator.u_min = 1.5;
        control.allocator.D_prop = 0.17;
        control.allocator.n_max_rpm = 3500.0;
        control.allocator.max_thrust_N = 250.0;
        control.allocator.KT_0 = 0.4566;

        control.motor.rho = 1028.0;
        control.motor.D_prop = 0.17;
        control.motor.rpm_max = 3500.0;
        control.motor.KT_0 = 0.4566;
        control.motor.KQ_0 = 0.07;

        control.gnc.depth.kp = 0.08;
        control.gnc.depth.kd = 0.18;
        control.gnc.depth.theta_max = 0.12;
        control.gnc.pitch.kp = 50.0;
        control.gnc.pitch.kd = 45.0;
        control.gnc.pitch.tau_max = 30.0;
        control.gnc.surge.kp = 220.0;
        control.gnc.surge.ki = 10.0;
        control.gnc.surge.kd = 14.0;

        profile.runtime.estimation_profile =
            hydrox::estimation_profile_for(hydrox::VehicleClass::UUV);
        profile.runtime.nominal_dt_s = 0.01;
        profile.runtime.max_sensor_dt_s = 0.25;
        profile.runtime.sensor_timeout_us = 500000;
        profile.runtime.setpoint_timeout_us = 500000;
        profile.runtime.motor = control.motor;
        profile.sensors = hydrox::SensorAdapter::Params(
            profile.runtime.estimation_profile);
        return true;
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
            "stream_backpressure", "configuration_error"
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
    mutable bool command_connected_ = false;
    mutable bool pending_command_ = false;
    mutable bool have_sender_generation_ = false;
    mutable bool have_sequence_ = false;
};

FmuV6cHitlBoard g_board;
}

extern "C" hydrox::runtime::HitlBoard *hydrox_hitl_board() noexcept
{
    return &g_board;
}