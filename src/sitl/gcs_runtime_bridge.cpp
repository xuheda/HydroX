#include "sitl/gcs_runtime_bridge.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

namespace hydrox::sitl
{
GcsRuntimeBridge::GcsRuntimeBridge(UdpSender &transport,
                                   uint8_t system_id,
                                   uint8_t component_id,
                                   uint8_t mav_type)
    : transport_(transport),
      protocol_(system_id, component_id, mav_type)
{
    telemetry_.mav_type = mav_type;
}

void GcsRuntimeBridge::set_link_connected(bool connected)
{
    telemetry_.link_connected = connected;
}

void GcsRuntimeBridge::update_telemetry(
    const runtime::HilRuntimeTick &tick,
    bool gps_valid,
    const HilGpsMsg &gps)
{
    const NavigationState &state = tick.estimated_state;
    telemetry_.time_boot_ms = static_cast<uint32_t>(
        tick.sensor_time_us / 1000ULL);
    telemetry_.mode = tick.mode;
    telemetry_.armed = tick.armed;
    telemetry_.actuator_authorized = tick.actuator_authorized;
    telemetry_.ekf_initialized = tick.ekf_initialized;
    telemetry_.gps_valid = gps_valid;

    telemetry_.roll_rad = static_cast<float>(state.eta[3]);
    telemetry_.pitch_rad = static_cast<float>(state.eta[4]);
    telemetry_.yaw_rad = static_cast<float>(state.eta[5]);
    telemetry_.roll_rate_radps = static_cast<float>(state.nu[3]);
    telemetry_.pitch_rate_radps = static_cast<float>(state.nu[4]);
    telemetry_.yaw_rate_radps = static_cast<float>(state.nu[5]);

    telemetry_.position_n_m = static_cast<float>(state.eta[0]);
    telemetry_.position_e_m = static_cast<float>(state.eta[1]);
    telemetry_.position_d_m = static_cast<float>(state.eta[2]);
    const double cosine = std::cos(state.eta[5]);
    const double sine = std::sin(state.eta[5]);
    const double velocity_n = cosine * state.nu[0] - sine * state.nu[1];
    const double velocity_e = sine * state.nu[0] + cosine * state.nu[1];
    telemetry_.velocity_n_mps = static_cast<float>(velocity_n);
    telemetry_.velocity_e_mps = static_cast<float>(velocity_e);
    telemetry_.velocity_d_mps = static_cast<float>(state.nu[2]);
    telemetry_.ground_speed_mps = static_cast<float>(
        std::hypot(velocity_n, velocity_e));
    telemetry_.equivalent_airspeed_mps = state.airspeed_valid
        ? static_cast<float>(state.equivalent_airspeed_mps)
        : std::numeric_limits<float>::quiet_NaN();

    if (gps_valid)
    {
        telemetry_.latitude_deg7 = gps.lat;
        telemetry_.longitude_deg7 = gps.lon;
        telemetry_.altitude_msl_mm = gps.alt;
    }

    telemetry_.battery_remaining_pct = tick.energy.soc >= 0.0
                                            ? static_cast<float>(
                                                  tick.energy.soc * 100.0)
                                            : -1.0f;
    float maximum_output = 0.0f;
    for (float output : tick.actuator.ch)
        maximum_output = std::max(maximum_output, std::abs(output));
    telemetry_.throttle_pct = static_cast<uint16_t>(
        std::clamp(maximum_output * 100.0f, 0.0f, 100.0f));
}

bool GcsRuntimeBridge::send_packet(void *context,
                                   const uint8_t *data,
                                   std::size_t size)
{
    return static_cast<GcsRuntimeBridge *>(context)->transport_.send(data, size);
}

void GcsRuntimeBridge::handle_command(void *context,
                                      const gcs::Command &command)
{
    static_cast<GcsRuntimeBridge *>(context)->apply_command(command);
}

void GcsRuntimeBridge::apply_command(const gcs::Command &command)
{
    if (active_runtime_ == nullptr)
        return;

    gcs::CommandResult result = gcs::CommandResult::TEMPORARILY_REJECTED;
    if (command.kind == gcs::CommandKind::SET_ARMED)
    {
        result = active_runtime_->set_armed(command.armed, active_now_us_)
                     ? gcs::CommandResult::ACCEPTED
                     : gcs::CommandResult::TEMPORARILY_REJECTED;
        if (result != gcs::CommandResult::ACCEPTED)
            (void)send_statustext(4, "Arm rejected: sensor not ready");
    }
    else if (command.kind == gcs::CommandKind::RESUME_CONTROL)
    {
        result = active_runtime_->resume_external_control(active_now_us_)
            ? gcs::CommandResult::ACCEPTED : gcs::CommandResult::TEMPORARILY_REJECTED;
        if (result != gcs::CommandResult::ACCEPTED)
            (void)send_statustext(4, "Resume rejected: fresh command/navigation required");
    }
    else if (command.kind == gcs::CommandKind::SET_MODE)
    {
        if (command.mode == GNCMode::DISABLED)
        {
            (void)active_runtime_->revoke_setpoint(active_now_us_);
            result = gcs::CommandResult::ACCEPTED;
        }
        else if (command.mode == GNCMode::WAYPOINT_3D)
        {
            // A waypoint mode transition requires a target from Mission or
            // Guided protocol. Never reuse an old waypoint implicitly.
            result = gcs::CommandResult::DENIED;
            (void)send_statustext(4, "Waypoint mode requires a mission target");
        }
        else
        {
            GNCSetpoint setpoint = active_runtime_->setpoint();
            setpoint.depth_ref = telemetry_.position_d_m;
            setpoint.heading_ref = telemetry_.yaw_rad;
            setpoint.surge_ref = 0.0;
            setpoint.use_yaw_rate_ref = false;
            setpoint.yaw_rate_ref = 0.0;
            setpoint.wp_n = telemetry_.position_n_m;
            setpoint.wp_e = telemetry_.position_e_m;
            setpoint.wp_d = telemetry_.position_d_m;
            setpoint.use_path_segment = false;
            setpoint.hold_heading = true;
            if (command.mode == GNCMode::SURFACE)
                setpoint.depth_ref = 0.0;

            result = active_runtime_->accept_setpoint(
                         setpoint, command.mode, active_now_us_)
                         ? gcs::CommandResult::ACCEPTED
                         : gcs::CommandResult::TEMPORARILY_REJECTED;
            if (result != gcs::CommandResult::ACCEPTED)
                (void)send_statustext(4, "Mode rejected: control gate not ready");
        }
    }

    (void)protocol_.acknowledge(command, result, this, send_packet);
}

void GcsRuntimeBridge::service(runtime::HilRuntime &runtime, uint64_t now_us)
{
    active_runtime_ = &runtime;
    active_now_us_ = now_us;
    telemetry_.monotonic_time_us = now_us;
    telemetry_.armed = runtime.armed();
    telemetry_.mode = runtime.mode();

    std::array<uint8_t, 2048> buffer{};
    for (int datagrams = 0; datagrams < 16; ++datagrams)
    {
        const int received = transport_.receive(buffer.data(), buffer.size());
        if (received <= 0)
            break;
        const bool learning_first_peer = !transport_.has_peer();
        const uint64_t before = protocol_.received_message_count();
        protocol_.ingest(buffer.data(), static_cast<std::size_t>(received),
                         now_us, this, handle_command, this, send_packet);
        if (protocol_.received_message_count() > before)
        {
            transport_.accept_last_peer();
            if (learning_first_peer && transport_.has_peer())
                std::printf("[FC] QGC MAVLink peer accepted\n");
        }
    }

    protocol_.update(telemetry_, this, send_packet);
    active_runtime_ = nullptr;
}

bool GcsRuntimeBridge::send_statustext(uint8_t severity, const char *text)
{
    return protocol_.send_statustext(severity, text, this, send_packet);
}
} // namespace hydrox::sitl
