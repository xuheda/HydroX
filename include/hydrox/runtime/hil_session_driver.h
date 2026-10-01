#pragma once

#include "hydrox/runtime/hil_runtime.h"
#include "mavlink_hil.h"
#include "sensor_adapter.h"

#include <cstdint>

namespace hydrox::runtime
{
    /**
     * Transport-neutral HIL session envelope shared by host SITL and embedded
     * HITL. TCP/serial ownership stays outside this class; connection epochs,
     * pause semantics, sensor timestamp epochs, runtime maintenance and the
     * requirement to publish a fresh actuator frame live here.
     */
    struct HilSessionResult
    {
        NavigationInput sensor_input{};
        RuntimeEvent runtime_event = RuntimeEvent::NONE;
        StepStatus step_status = StepStatus::OK;
        uint64_t generation = 0;
        bool has_sensor_input = false;
        bool sensor_prepared = false;
        bool sensor_rejected = false;
        bool sensor_ready = false;
        bool sensor_epoch_restarted = false;
        bool pause_changed = false;
        bool simulator_paused = false;
        bool actuator_frame_required = false;
        bool egress_reset_required = false;
    };

    class HilSessionDriver
    {
    public:
        HilSessionDriver(MavlinkHIL &codec,
                         HilRuntime &runtime,
                         const SensorAdapter::Params &sensor_params);

        HilSessionResult on_connected(platform::MonotonicTimeUs now_us);
        HilSessionResult on_disconnected(platform::MonotonicTimeUs now_us);

        /** Ingest one decoded MAVLink frame without executing a control tick. */
        HilSessionResult ingest_frame(
            const MavFrame &frame,
            platform::MonotonicTimeUs now_us);

        /**
         * Validate a timestamped sensor input and open the command gate. The
         * caller may service its platform command transport before step_sensor().
         */
        HilSessionResult prepare_sensor(
            const NavigationInput &input,
            platform::MonotonicTimeUs now_us);

        /** Execute the sensor tick prepared by prepare_sensor(). */
        HilSessionResult step_sensor(platform::MonotonicTimeUs now_us);

        /** Convenience path for supervisors without interposed command work. */
        HilSessionResult process_sensor(
            const NavigationInput &input,
            platform::MonotonicTimeUs now_us);

        HilSessionResult maintain(platform::MonotonicTimeUs now_us);
        HilSessionResult revoke_setpoint(platform::MonotonicTimeUs now_us);

        bool accept_setpoint(
            const GNCSetpoint &setpoint,
            GNCMode mode,
            platform::MonotonicTimeUs received_at_us);

        NavigationInput navigation_snapshot() const;
        bool simulator_paused() const noexcept { return simulator_paused_; }

    private:
        void reset_link_state();

        MavlinkHIL &codec_;
        HilRuntime &runtime_;
        SensorAdapter sensor_adapter_;
        NavigationInput prepared_input_{};
        uint64_t last_sensor_time_us_ = 0;
        platform::MonotonicTimeUs last_sensor_arrival_us_ = 0;
        bool prepared_sensor_ = false;
        bool simulator_paused_ = false;
    };
} // namespace hydrox::runtime
