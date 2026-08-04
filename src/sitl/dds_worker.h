#pragma once

#include "dds_connection_state.h"
#include "dds_setpoint_codec.h"
#include "fc_snapshot.h"
#include "latest_value_mailbox.h"
#include "mavlink_hil.h"
#include "hydrox/platform/clock.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace hydrox::sitl
{
    struct DdsWorkerConfig
    {
        std::string agent_ip = "127.0.0.1";
        uint16_t agent_port = 8888;
        uint16_t domain_id = 0;
        std::string vehicle = "vehicle0";
        uint32_t client_key = 1;
        bool publish_truth_state = false;
    };

    struct DdsTelemetrySample
    {
        FcSnapshot snapshot{};
        std::string gnc_mode = "DISABLED";
        bool hil_connected = false;
        bool ekf_initialized = false;
        bool actuator_authorized = false;
        HilPassiveSonarBearingMsg passive_sonar{};
        HilAcousticNeighborsMsg acoustic_neighbors{};
        HilRangeFinderScanMsg rangefinder_scan{};
    };

    struct DdsSetpointSample
    {
        GNCSetpointDds setpoint{};
        uint64_t session_generation = 0;
        platform::MonotonicTimeUs received_at_us = 0;
    };

    // Owns all Micro XRCE-DDS operations on one background thread. The GNC
    // thread only touches non-blocking latest-value mailboxes.
    class DdsWorker
    {
    public:
        DdsWorker(DdsWorkerConfig config, const platform::Clock &clock);
        ~DdsWorker();

        DdsWorker(const DdsWorker &) = delete;
        DdsWorker &operator=(const DdsWorker &) = delete;

        bool try_submit_telemetry(DdsTelemetrySample sample);
        bool try_take_setpoint(uint64_t &last_sequence, DdsSetpointSample &sample);
        bool try_take_connection_status(uint64_t &last_sequence,
                                        DdsConnectionStatus &status);
        bool is_connected() const;

    private:
        void run();
        bool wait_or_stop(std::chrono::milliseconds duration);
        void publish_connection_status(DdsConnectionState state,
                                       uint32_t connection_attempt);

        DdsWorkerConfig config_;
        const platform::Clock &clock_;
        LatestValueMailbox<DdsTelemetrySample> telemetry_mailbox_;
        LatestValueMailbox<DdsSetpointSample> setpoint_mailbox_;
        LatestValueMailbox<DdsConnectionStatus> connection_status_mailbox_;
        std::atomic<bool> running_{true};
        std::atomic<DdsConnectionState> connection_state_{
            DdsConnectionState::DISCONNECTED};
        uint64_t session_generation_ = 0;
        uint64_t disconnect_epoch_ = 0;
        std::mutex wake_mutex_;
        std::condition_variable wake_cv_;
        std::thread thread_;
    };
} // namespace hydrox::sitl
