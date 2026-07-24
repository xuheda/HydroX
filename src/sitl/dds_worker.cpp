#include "dds_worker.h"

#include "dds_publisher.h"

#include <chrono>
#include <cstdio>
#include <exception>
#include <memory>
#include <utility>

namespace hydrox::sitl
{
    using namespace std::chrono_literals;

    DdsWorker::DdsWorker(DdsWorkerConfig config)
        : config_(std::move(config)),
          thread_(&DdsWorker::run, this)
    {
    }

    DdsWorker::~DdsWorker()
    {
        running_ = false;
        wake_cv_.notify_all();
        if (thread_.joinable())
            thread_.join();
    }

    bool DdsWorker::try_submit_telemetry(DdsTelemetrySample sample)
    {
        const bool accepted = telemetry_mailbox_.try_publish(std::move(sample));
        if (accepted)
            wake_cv_.notify_one();
        return accepted;
    }

    bool DdsWorker::try_take_setpoint(uint64_t &last_sequence,
                                      DdsSetpointSample &sample)
    {
        return setpoint_mailbox_.try_take_if_new(last_sequence, sample);
    }

    bool DdsWorker::try_take_connection_status(
        uint64_t &last_sequence,
        DdsConnectionStatus &status)
    {
        return connection_status_mailbox_.try_take_if_new(last_sequence, status);
    }

    bool DdsWorker::is_connected() const
    {
        return connection_state_.load(std::memory_order_acquire) ==
               DdsConnectionState::CONNECTED;
    }

    bool DdsWorker::wait_or_stop(std::chrono::milliseconds duration)
    {
        std::unique_lock<std::mutex> lock(wake_mutex_);
        wake_cv_.wait_for(lock, duration, [this]() { return !running_; });
        return running_;
    }

    void DdsWorker::publish_connection_status(
        DdsConnectionState state,
        uint32_t connection_attempt)
    {
        connection_state_.store(state, std::memory_order_release);
        connection_status_mailbox_.publish(DdsConnectionStatus{
            state,
            session_generation_,
            disconnect_epoch_,
            connection_attempt,
        });
    }

    void DdsWorker::run()
    {
        std::unique_ptr<DdsPublisher> publisher;
        uint64_t last_telemetry_sequence = 0;
        uint32_t connection_attempt = 0;

        publish_connection_status(DdsConnectionState::DISCONNECTED, 0);

        while (running_)
        {
            if (!publisher)
            {
                ++connection_attempt;
                publish_connection_status(
                    DdsConnectionState::CONNECTING, connection_attempt);
                if (connection_attempt <= 3 || connection_attempt % 10 == 0)
                {
                    std::printf("[FC][DDS] background connect attempt #%u to %s:%u\n",
                                static_cast<unsigned>(connection_attempt),
                                config_.agent_ip.c_str(),
                                static_cast<unsigned>(config_.agent_port));
                }

                try
                {
                    auto candidate = std::make_unique<DdsPublisher>(
                        config_.agent_ip,
                        config_.agent_port,
                        config_.domain_id,
                        config_.vehicle,
                        config_.client_key,
                        config_.publish_truth_state);
                    if (candidate->is_connected())
                    {
                        ++session_generation_;
                        const uint64_t callback_generation = session_generation_;
                        candidate->set_setpoint_callback(
                            [this, callback_generation](const GNCSetpointDds &setpoint)
                            {
                                // Blocking is allowed on the DDS thread. The
                                // GNC reader always uses try_lock.
                                 setpoint_mailbox_.publish(DdsSetpointSample{
                                     setpoint,
                                     callback_generation,
                                     std::chrono::steady_clock::now(),
                                 });
                            });
                        publisher = std::move(candidate);
                        publish_connection_status(
                            DdsConnectionState::CONNECTED, 0);
                        connection_attempt = 0;
                        std::printf(
                            "[FC][DDS] background worker connected session=%llu\n",
                            static_cast<unsigned long long>(session_generation_));
                    }
                }
                catch (const std::exception &error)
                {
                    std::fprintf(stderr,
                                 "[FC][DDS] background connect exception: %s\n",
                                 error.what());
                }

                if (!publisher)
                {
                    const auto retry_delay =
                        dds_reconnect_delay(connection_attempt);
                    publish_connection_status(
                        DdsConnectionState::BACKOFF, connection_attempt);
                    if (!wait_or_stop(retry_delay))
                        break;
                    continue;
                }
            }

            DdsTelemetrySample sample;
            const bool have_telemetry = telemetry_mailbox_.try_take_if_new(
                last_telemetry_sequence, sample);
            if (have_telemetry)
            {
                publisher->publish_fc_snapshot(
                    sample.snapshot,
                    sample.gnc_mode,
                    sample.hil_connected,
                    sample.ekf_initialized);
                publisher->publish_passive_sonar_bearing(sample.passive_sonar);
                publisher->publish_acoustic_neighbors(sample.acoustic_neighbors);
                publisher->publish_rangefinder_scan(sample.rangefinder_scan);
            }

            if (!publisher->spin_once())
            {
                ++disconnect_epoch_;
                connection_attempt = 0;
                publish_connection_status(DdsConnectionState::BACKOFF, 1);
                std::fprintf(stderr,
                             "[FC][DDS] Agent health check failed; reconnecting in background\n");
                publisher.reset();
                if (!wait_or_stop(dds_reconnect_delay(1)))
                    break;
                continue;
            }

            if (!have_telemetry && !wait_or_stop(2ms))
                break;
        }

        publisher.reset();
        publish_connection_status(DdsConnectionState::STOPPED, 0);
    }
} // namespace hydrox::sitl
