#pragma once
/**
 * dds_publisher.h — HydroX SITL Micro XRCE-DDS bridge.
 *
 * The public header contains only the publisher contract and state layout.
 * Entity creation and handwritten CDR serialization live in dds_publisher.cpp.
 */

#ifdef HYDROX_DDS_ENABLED

#include <ucdr/microcdr.h>
#include <uxr/client/client.h>

#include <chrono>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

#include "dds_setpoint_codec.h"
#include "dds_cdr_writer.h"
#include "dds_topic_manifest.h"
#include "fc_snapshot.h"
#include "mavlink_hil.h"

namespace hydrox
{
    using SetpointCallback = std::function<void(const GNCSetpointDds &)>;

    class DdsPublisher
    {
    public:
        DdsPublisher(const std::string &agent_ip,
                     uint16_t agent_port,
                     uint16_t domain_id,
                     const std::string &vehicle,
                     uint32_t key,
                     bool publish_truth_state = false);
        ~DdsPublisher();

        DdsPublisher(const DdsPublisher &) = delete;
        DdsPublisher &operator=(const DdsPublisher &) = delete;

        bool is_connected() const;
        void set_setpoint_callback(SetpointCallback cb);

        void publish_fc_snapshot(const FcSnapshot &fs,
                                 const std::string &gnc_mode_str,
                                 bool hil_connected,
                                 bool ekf_init);
        void publish_passive_sonar_bearing(const HilPassiveSonarBearingMsg &bearing);
        void publish_acoustic_neighbors(const HilAcousticNeighborsMsg &neighbors);
        void publish_rangefinder_scan(const HilRangeFinderScanMsg &scan);

        // Poll the setpoint subscription and reliable-stream health.
        // False asks the owner to reconstruct this publisher and reconnect.
        bool spin_once();

    private:
        uxrUDPTransport transport_{};
        uxrSession session_{};
        uint8_t out_buf_[UXR_CONFIG_UDP_TRANSPORT_MTU * 16]{};
        uint8_t out_best_effort_buf_[UXR_CONFIG_UDP_TRANSPORT_MTU * 16]{};
        uint8_t in_reliable_buf_[UXR_CONFIG_UDP_TRANSPORT_MTU * 16]{};
        uxrStreamId out_reliable_{};
        uxrStreamId out_best_effort_{};
        uxrStreamId in_reliable_{};
        uxrStreamId in_best_effort_{};
        bool transport_initialized_{false};
        bool session_created_{false};
        bool connected_{false};
        uint32_t health_miss_count_{0};
        std::chrono::steady_clock::time_point next_health_check_{};
        // A shared local Agent serves every vehicle.  A short scheduling pause
        // must not make all clients reconnect at once and amplify the pause.
        static constexpr uint32_t HEALTH_MISS_LIMIT = 5;

        uxrObjectId participant_id_{};
        uxrObjectId pub_id_{};
        uxrObjectId dw_local_pos_id_{};
        uxrObjectId dw_sensor_id_{};
        uxrObjectId dw_actuator_id_{};
        uxrObjectId dw_status_id_{};
        uxrObjectId dw_state_estimate_id_{};
        uxrObjectId dw_truth_state_id_{};
        uxrObjectId dw_odom_id_{};
        uxrObjectId dw_tf_id_{};
        uxrObjectId dw_passive_sonar_bearing_id_{};
        uxrObjectId dw_acoustic_neighbors_id_{};
        uxrObjectId dw_rangefinder_scan_id_{};
        uxrObjectId sub_id_{};
        uxrObjectId dr_setpoint_id_{};

        uint16_t domain_id_{0};
        std::string vehicle_;
        bool publish_truth_state_{false};
        SetpointCallback sp_cb_;
        uint32_t rejected_setpoint_count_{0};
        uint32_t serialization_failure_count_{0};
        uint32_t stream_full_failure_count_{0};
        uint64_t last_status_pub_us_{0};
        uint64_t last_snapshot_pub_us_{0};
        uint64_t last_diagnostics_pub_us_{0};
        uint64_t last_truth_state_pub_us_{0};
        uint64_t last_odom_tf_pub_us_{0};
        uint64_t last_passive_sonar_pub_us_{0};
        uint64_t last_acoustic_neighbors_pub_us_{0};
        uint64_t last_rangefinder_scan_pub_us_{0};

        static constexpr uint64_t STATUS_PERIOD_US =
            dds_topics::period_us(dds_topics::kVehicleStatus);
        static constexpr uint64_t SNAPSHOT_PERIOD_US =
            dds_topics::period_us(dds_topics::kStateEstimate);
        static constexpr uint64_t DIAGNOSTICS_PERIOD_US =
            dds_topics::period_us(dds_topics::kVehicleLocalPosition);
        static constexpr uint64_t TRUTH_STATE_PERIOD_US =
            dds_topics::period_us(dds_topics::kTruthState);
        static constexpr uint64_t ODOM_TF_PERIOD_US =
            dds_topics::period_us(dds_topics::kOdometry);
        // The largest current payload is nav_msgs/Odometry (two 36-element
        // covariance arrays). Keep one reusable, bounded scratch buffer on the
        // DDS worker; overflow is detected and the sample is never enqueued.
        std::array<uint8_t, dds_cdr::MAX_TOPIC_PAYLOAD_BYTES>
            cdr_payload_buffer_{};

        void init_entities();
        void on_topic_received(ucdrBuffer *ub, uint16_t len);
        bool wait_entity_created(const char *entity, uint16_t req);

        bool enqueue_serialized_topic(uxrStreamId stream,
                                      uxrObjectId writer,
                                      const char *topic_name,
                                      uint8_t *payload,
                                      size_t payload_size);
        void log_publish_failure(const char *topic_name,
                                 const char *reason,
                                 uint32_t &counter);

        template <typename SerializeFn>
        bool buffer_topic(uxrStreamId stream,
                          uxrObjectId writer,
                          const char *topic_name,
                          SerializeFn &&serialize)
        {
            dds_cdr::Writer cdr(
                cdr_payload_buffer_.data(), cdr_payload_buffer_.size());
            serialize(cdr);
            if (!cdr.ok() || cdr.size() == 0)
            {
                log_publish_failure(
                    topic_name, "CDR serialization overflow", serialization_failure_count_);
                return false;
            }
            return enqueue_serialized_topic(
                stream, writer, topic_name, cdr.data(), cdr.size());
        }

        uxrObjectId create_participant(uint16_t id);
        uxrObjectId create_topic(uint16_t id,
                                 const std::string &topic_name,
                                 const std::string &type_name);
        uxrObjectId create_publisher(uint16_t id);
        uxrObjectId create_subscriber(uint16_t id);
        uxrObjectId create_datawriter(uint16_t id,
                                      uxrObjectId pub_id,
                                      uxrObjectId topic_id);
        uxrObjectId create_datareader(uint16_t id,
                                      uxrObjectId sub_id,
                                      uxrObjectId topic_id);
    };

} // namespace hydrox

#endif // HYDROX_DDS_ENABLED
