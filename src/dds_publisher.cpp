#include "dds_publisher.h"

#ifdef HYDROX_DDS_ENABLED

#include <uxr/client/core/session/create_entities_bin.h>
#include <uxr/client/util/ping.h>

#include "dds_cdr_helpers.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

namespace hydrox
{
    DdsPublisher::DdsPublisher(const std::string &agent_ip,
                               uint16_t agent_port,
                               uint16_t domain_id,
                               const std::string &vehicle,
                               uint32_t key,
                               bool publish_truth_state)
        : domain_id_(domain_id),
          vehicle_(vehicle),
          publish_truth_state_(publish_truth_state)
    {
        const std::string agent_port_str = std::to_string(agent_port);
        if (!uxr_init_udp_transport(&transport_, UXR_IPv4,
                                    agent_ip.c_str(), agent_port_str.c_str()))
        {
            std::printf("[FC][DDS] transport init failed\n");
            return;
        }
        transport_initialized_ = true;

        uxr_init_session(&session_, &transport_.comm, key);
        session_.on_topic = [](uxrSession *, uxrObjectId, uint16_t,
                               uxrStreamId, struct ucdrBuffer *ub,
                               uint16_t len, void *args)
        {
            static_cast<DdsPublisher *>(args)->on_topic_received(ub, len);
        };
        session_.on_topic_args = this;

        session_created_ = uxr_create_session_retries(&session_, 1);
        if (!session_created_)
        {
            std::printf("[FC][DDS] create session failed\n");
            return;
        }
        connected_ = true;

        out_reliable_ = uxr_create_output_reliable_stream(
            &session_, out_buf_, sizeof(out_buf_), 16);
        out_best_effort_ = uxr_create_output_best_effort_stream(
            &session_, out_best_effort_buf_, sizeof(out_best_effort_buf_));
        in_reliable_ = uxr_create_input_reliable_stream(
            &session_, in_reliable_buf_, sizeof(in_reliable_buf_), 16);
        in_best_effort_ = uxr_create_input_best_effort_stream(&session_);

        init_entities();
        if (connected_)
        {
            next_health_check_ = std::chrono::steady_clock::now() +
                                 std::chrono::milliseconds(500);
            std::printf("[FC][DDS] ready vehicle=%s key=%u domain=%u agent=%s:%u\n",
                        vehicle_.c_str(), static_cast<unsigned>(key),
                        static_cast<unsigned>(domain_id_),
                        agent_ip.c_str(), static_cast<unsigned>(agent_port));
            if (publish_truth_state_)
            {
                std::printf(
                    "[FC][DDS] debug truth_state publishing enabled for vehicle=%s\n",
                    vehicle_.c_str());
            }
        }
    }

    DdsPublisher::~DdsPublisher()
    {
        // Never wait for an unavailable Agent during worker shutdown/reconnect.
        if (session_created_)
            uxr_delete_session_retries(&session_, 0);
        if (transport_initialized_)
            uxr_close_udp_transport(&transport_);
    }

    bool DdsPublisher::is_connected() const
    {
        return connected_;
    }

    void DdsPublisher::set_setpoint_callback(SetpointCallback cb)
    {
        sp_cb_ = std::move(cb);
    }

    void DdsPublisher::log_publish_failure(
        const char *topic_name,
        const char *reason,
        uint32_t &counter)
    {
        ++counter;
        if (counter <= 10 || counter % 100 == 0)
        {
            std::fprintf(
                stderr,
                "[FC][DDS] publish rejected topic=%s reason=%s count=%u\n",
                topic_name ? topic_name : "unknown",
                reason ? reason : "unknown",
                static_cast<unsigned>(counter));
        }
    }

    bool DdsPublisher::enqueue_serialized_topic(
        uxrStreamId stream,
        uxrObjectId writer,
        const char *topic_name,
        uint8_t *payload,
        size_t payload_size)
    {
        uint16_t request = uxr_buffer_topic(
            &session_, stream, writer, payload, payload_size);
        if (request != UXR_INVALID_REQUEST_ID)
            return true;

        // Best-effort space is reclaimed by flushing. Reliable streams may
        // additionally need to process an ACK before a slot becomes reusable.
        uxr_flash_output_streams(&session_);
        if (stream.type == UXR_RELIABLE_STREAM)
            uxr_run_session_time(&session_, 1);

        request = uxr_buffer_topic(
            &session_, stream, writer, payload, payload_size);
        if (request != UXR_INVALID_REQUEST_ID)
            return true;

        log_publish_failure(
            topic_name, "XRCE output stream full", stream_full_failure_count_);
        return false;
    }

    bool DdsPublisher::wait_entity_created(const char *entity, uint16_t req)
    {
        if (0 == req)
        {
            std::printf("[FC][DDS] %s request rejected\n", entity);
            return false;
        }

        uint8_t status[1] = {UXR_STATUS_NONE};
        if (!uxr_run_session_until_all_status(&session_, 1000, &req, status, 1))
        {
            std::printf("[FC][DDS] %s status timeout req=%u status=%u\n",
                        entity, req, static_cast<unsigned>(status[0]));
            return false;
        }

        if (UXR_STATUS_OK != status[0] && UXR_STATUS_OK_MATCHED != status[0])
        {
            std::printf("[FC][DDS] %s failed req=%u status=%u\n",
                        entity, req, static_cast<unsigned>(status[0]));
            return false;
        }

        return true;
    }

    uxrObjectId DdsPublisher::create_participant(uint16_t id)
    {
        uxrObjectId oid = uxr_object_id(id, UXR_PARTICIPANT_ID);
        const std::string participant_name = "hydrox_" + vehicle_ + "_participant";
        uint16_t req = uxr_buffer_create_participant_bin(
            &session_, out_reliable_, oid, domain_id_, participant_name.c_str(), UXR_REPLACE);
        return wait_entity_created("create participant", req)
                   ? oid
                   : uxr_object_id(0, UXR_PARTICIPANT_ID);
    }

    uxrObjectId DdsPublisher::create_topic(uint16_t id,
                                                  const std::string &topic_name,
                                                  const std::string &type_name)
    {
        uxrObjectId oid = uxr_object_id(id, UXR_TOPIC_ID);
        uint16_t req = uxr_buffer_create_topic_bin(
            &session_, out_reliable_, oid, participant_id_,
            topic_name.c_str(), type_name.c_str(), UXR_REPLACE);
        return wait_entity_created("create topic", req)
                   ? oid
                   : uxr_object_id(0, UXR_TOPIC_ID);
    }

    uxrObjectId DdsPublisher::create_publisher(uint16_t id)
    {
        uxrObjectId oid = uxr_object_id(id, UXR_PUBLISHER_ID);
        uint16_t req = uxr_buffer_create_publisher_bin(
            &session_, out_reliable_, oid, participant_id_, UXR_REPLACE);
        return wait_entity_created("create publisher", req)
                   ? oid
                   : uxr_object_id(0, UXR_PUBLISHER_ID);
    }

    uxrObjectId DdsPublisher::create_subscriber(uint16_t id)
    {
        uxrObjectId oid = uxr_object_id(id, UXR_SUBSCRIBER_ID);
        uint16_t req = uxr_buffer_create_subscriber_bin(
            &session_, out_reliable_, oid, participant_id_, UXR_REPLACE);
        return wait_entity_created("create subscriber", req)
                   ? oid
                   : uxr_object_id(0, UXR_SUBSCRIBER_ID);
    }

    uxrObjectId DdsPublisher::create_datawriter(uint16_t id,
                                                       uxrObjectId pub_id,
                                                       uxrObjectId topic_id)
    {
        uxrObjectId oid = uxr_object_id(id, UXR_DATAWRITER_ID);
        uxrQoS_t qos{};
        qos.durability = UXR_DURABILITY_TRANSIENT_LOCAL;
        qos.reliability = UXR_RELIABILITY_BEST_EFFORT;
        qos.history = UXR_HISTORY_KEEP_LAST;
        qos.depth = 0; // PX4-style: omit explicit depth, use agent/default history.
        uint16_t req = uxr_buffer_create_datawriter_bin(
            &session_, out_reliable_, oid, pub_id, topic_id, qos, UXR_REPLACE);
        return wait_entity_created("create datawriter", req)
                   ? oid
                   : uxr_object_id(0, UXR_DATAWRITER_ID);
    }

    uxrObjectId DdsPublisher::create_datareader(uint16_t id,
                                                       uxrObjectId sub_id,
                                                       uxrObjectId topic_id)
    {
        uxrObjectId oid = uxr_object_id(id, UXR_DATAREADER_ID);
        uxrQoS_t qos{};
        qos.durability = UXR_DURABILITY_VOLATILE;
        qos.reliability = UXR_RELIABILITY_BEST_EFFORT;
        qos.history = UXR_HISTORY_KEEP_LAST;
        qos.depth = 1;
        uint16_t req = uxr_buffer_create_datareader_bin(
            &session_, out_reliable_, oid, sub_id, topic_id, qos, UXR_REPLACE);
        // Request message forwarding from the agent
        uxrDeliveryControl delivery_control = {0};
        delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
        delivery_control.max_elapsed_time = UXR_MAX_ELAPSED_TIME_UNLIMITED;
        delivery_control.max_bytes_per_second = UXR_MAX_BYTES_PER_SECOND_UNLIMITED;
        delivery_control.min_pace_period = 0;
        uxr_buffer_request_data(&session_, out_reliable_, oid,
                                in_reliable_, &delivery_control);
        return wait_entity_created("create datareader", req)
                   ? oid
                   : uxr_object_id(0, UXR_DATAREADER_ID);
    }

    void DdsPublisher::init_entities()
    {
        participant_id_ = create_participant(1);
        if (0 == participant_id_.id)
        {
            connected_ = false;
            return;
        }

        const auto fail_if_invalid = [this](uxrObjectId oid) -> bool
        {
            if (0 == oid.id)
            {
                connected_ = false;
                return true;
            }
            return false;
        };

        // Topic/type/id/rate definitions live in dds_topic_manifest.h.
        for (const auto &topic : dds_topics::kManifest)
        {
            if (!publish_truth_state_ && std::string(topic.label) == "truth_state")
                continue;

            const auto topic_id = create_topic(
                topic.topic_id,
                dds_topics::dds_name(topic, vehicle_),
                topic.type_name);
            if (fail_if_invalid(topic_id))
                return;
        }

        // ── Publisher / DataWriters ───────────────────────────────────────────
        pub_id_ = create_publisher(1);
        if (fail_if_invalid(pub_id_))
            return;

        const auto create_writer_for =
            [this](const dds_topics::TopicSpec &topic) -> uxrObjectId
        {
            return create_datawriter(
                topic.endpoint_id,
                pub_id_,
                uxr_object_id(topic.topic_id, UXR_TOPIC_ID));
        };

        dw_local_pos_id_ = create_writer_for(dds_topics::kVehicleLocalPosition);
        if (fail_if_invalid(dw_local_pos_id_))
            return;

        dw_sensor_id_ = create_writer_for(dds_topics::kSensorCombined);
        if (fail_if_invalid(dw_sensor_id_))
            return;

        dw_actuator_id_ = create_writer_for(dds_topics::kActuatorOutputs);
        if (fail_if_invalid(dw_actuator_id_))
            return;

        dw_status_id_ = create_writer_for(dds_topics::kVehicleStatus);
        if (fail_if_invalid(dw_status_id_))
            return;

        dw_auv_state_id_ = create_writer_for(dds_topics::kAuvState);
        if (fail_if_invalid(dw_auv_state_id_))
            return;

        if (publish_truth_state_)
        {
            dw_truth_state_id_ = create_writer_for(dds_topics::kTruthState);
            if (fail_if_invalid(dw_truth_state_id_))
                return;
        }

        dw_odom_id_ = create_writer_for(dds_topics::kOdometry);
        if (fail_if_invalid(dw_odom_id_))
            return;

        dw_tf_id_ = create_writer_for(dds_topics::kTf);
        if (fail_if_invalid(dw_tf_id_))
            return;

        // ── Subscriber / DataReader ───────────────────────────────────────────
        dw_passive_sonar_bearing_id_ =
            create_writer_for(dds_topics::kPassiveSonarBearing);
        if (fail_if_invalid(dw_passive_sonar_bearing_id_))
            return;

        dw_acoustic_neighbors_id_ =
            create_writer_for(dds_topics::kAcousticNeighbors);
        if (fail_if_invalid(dw_acoustic_neighbors_id_))
            return;

        dw_rangefinder_scan_id_ =
            create_writer_for(dds_topics::kRangeFinderScan);
        if (fail_if_invalid(dw_rangefinder_scan_id_))
            return;

        sub_id_ = create_subscriber(1);
        if (fail_if_invalid(sub_id_))
            return;

        dr_setpoint_id_ = create_datareader(
            dds_topics::kSetpoint.endpoint_id,
            sub_id_,
            uxr_object_id(dds_topics::kSetpoint.topic_id, UXR_TOPIC_ID));
        if (fail_if_invalid(dr_setpoint_id_))
            return;
    }

    // on_topic_received — Receive GNCSetpoint, CDR deserialization
    void DdsPublisher::on_topic_received(ucdrBuffer *ub, uint16_t len)
    {
        const auto reject_sample = [this](const char *reason)
        {
            ++rejected_setpoint_count_;
            if (rejected_setpoint_count_ <= 5 ||
                rejected_setpoint_count_ % 100 == 0)
            {
                std::fprintf(stderr,
                             "[FC][DDS] rejected GNCSetpoint #%u: %s\n",
                             static_cast<unsigned>(rejected_setpoint_count_),
                             reason);
            }
        };

        if (!ub || !ub->iterator || len == 0 ||
            static_cast<std::size_t>(len) > ucdr_buffer_remaining(ub))
        {
            reject_sample("invalid callback payload length");
            return;
        }

        GNCSetpointDds sp;
        std::string error;
        if (!dds_cdr::decode_gnc_setpoint(ub->iterator, len, sp, &error))
        {
            reject_sample(error.c_str());
            return;
        }

        if (sp_cb_)
            sp_cb_(sp);
    }

    // spin_once — Non-blocking polling on the dedicated DDS worker thread
    bool DdsPublisher::spin_once()
    {
        if (!connected_)
            return false;

        // Process queued writes and incoming Setpoints without waiting. Reliable
        // confirmation state alone is not an Agent liveness signal.
        uxr_run_session_time(&session_, 0);

        const auto now = std::chrono::steady_clock::now();
        if (now < next_health_check_)
            return true;
        next_health_check_ = now + std::chrono::milliseconds(500);

        if (uxr_ping_agent_session(&session_, 50, 1))
        {
            health_miss_count_ = 0;
            return true;
        }

        ++health_miss_count_;
        if (health_miss_count_ < HEALTH_MISS_LIMIT)
            return true;

        connected_ = false;
        return false;
    }

    void DdsPublisher::publish_passive_sonar_bearing(
        const HilPassiveSonarBearingMsg &bearing)
    {
        if (!connected_ || 0 == dw_passive_sonar_bearing_id_.id ||
            bearing.time_usec == 0 || bearing.time_usec == last_passive_sonar_pub_us_)
            return;

        const uint64_t stamp_ns = bearing.time_usec * 1000ULL;
        const std::string frame = vehicle_ + "/base_link_frd";
        const bool buffered = buffer_topic(
            out_best_effort_,
            dw_passive_sonar_bearing_id_,
            "PassiveSonarBearing",
            [&](dds_cdr::Writer &cdr)
            {
                cdr.header(stamp_ns, frame.c_str());
                cdr.string(vehicle_.c_str());
                cdr.uint64(stamp_ns);
                cdr.boolean(bearing.valid);
                cdr.uint32(bearing.target_slot);
                cdr.float32(bearing.azimuth_rad);
                cdr.float32(bearing.elevation_rad);
            });
        if (buffered)
            last_passive_sonar_pub_us_ = bearing.time_usec;
    }

    void DdsPublisher::publish_acoustic_neighbors(
        const HilAcousticNeighborsMsg &neighbors)
    {
        if (!connected_ || 0 == dw_acoustic_neighbors_id_.id ||
            neighbors.time_usec == 0 ||
            neighbors.time_usec == last_acoustic_neighbors_pub_us_)
            return;

        const uint64_t stamp_ns = neighbors.time_usec * 1000ULL;
        const std::string frame = vehicle_ + "/base_link_frd";
        const uint32_t count =
            static_cast<uint32_t>(std::min<size_t>(neighbors.contacts.size(), 4));

        const bool buffered = buffer_topic(
            out_best_effort_,
            dw_acoustic_neighbors_id_,
            "AcousticNeighbors",
            [&](dds_cdr::Writer &cdr)
            {
                cdr.header(stamp_ns, frame.c_str());
                cdr.uint64(neighbors.time_usec);
                cdr.uint32(neighbors.receiver_id);
                cdr.string(vehicle_.c_str());
                cdr.uint32(count);

                for (uint32_t i = 0; i < count; ++i)
                {
                    const auto &contact = neighbors.contacts[i];
                    cdr.boolean(contact.valid);
                    cdr.uint32(contact.sender_id);
                    cdr.float32(contact.range_m);
                    cdr.float32(contact.propagation_delay_s);
                    cdr.float32(contact.azimuth_rad);
                    cdr.float32(contact.elevation_rad);
                    cdr.float32(contact.depth_m);
                    for (float value : contact.position_ned)
                        cdr.float32(value);
                    for (float value : contact.velocity_ned)
                        cdr.float32(value);
                    for (float value : contact.attitude_rpy)
                        cdr.float32(value);
                    for (int k = 0; k < 3; ++k)
                        cdr.float32(0.0f);
                    for (int k = 0; k < 8; ++k)
                    {
                        const float value = (k < 3) ? contact.payload[k] : 0.0f;
                        cdr.float32(value);
                    }
                }
            });
        if (buffered)
            last_acoustic_neighbors_pub_us_ = neighbors.time_usec;
    }

    void DdsPublisher::publish_rangefinder_scan(
        const HilRangeFinderScanMsg &scan)
    {
        if (!connected_ || 0 == dw_rangefinder_scan_id_.id ||
            scan.time_usec == 0 || scan.time_usec == last_rangefinder_scan_pub_us_ ||
            !std::isfinite(scan.max_range_m) || scan.max_range_m <= 0.0f ||
            scan.ranges_m.empty())
            return;

        const uint64_t stamp_ns = scan.time_usec * 1000ULL;
        const std::string frame = vehicle_ + "/base_link_frd";
        const uint32_t count =
            static_cast<uint32_t>(std::min<size_t>(scan.ranges_m.size(), 48));
        const bool buffered = buffer_topic(
            out_best_effort_,
            dw_rangefinder_scan_id_,
            "RangefinderScan",
            [&](dds_cdr::Writer &cdr)
            {
                cdr.header(stamp_ns, frame.c_str());
                cdr.uint64(scan.time_usec);
                cdr.string(vehicle_.c_str());
                cdr.string("rangefinder_scan");
                cdr.uint32(count);
                cdr.float32(scan.max_range_m);

                cdr.uint32(count);
                for (uint32_t i = 0; i < count; ++i)
                {
                    const float value = std::isfinite(scan.ranges_m[i])
                                            ? scan.ranges_m[i]
                                            : -1.0f;
                    cdr.float32(value);
                }

                cdr.uint32(count);
                const float max_range = std::max(scan.max_range_m, 1.0e-6f);
                for (uint32_t i = 0; i < count; ++i)
                {
                    const float value = std::isfinite(scan.ranges_m[i])
                                            ? scan.ranges_m[i]
                                            : -1.0f;
                    const float normalized = value >= 0.0f
                        ? std::clamp(value / max_range, 0.0f, 1.0f)
                        : 1.0f;
                    cdr.float32(normalized);
                }
            });
        if (buffered)
            last_rangefinder_scan_pub_us_ = scan.time_usec;
    }

    // publish_fc_snapshot
    void DdsPublisher::publish_fc_snapshot(const FcSnapshot &fs,
                                                  const std::string &gnc_mode_str,
                                                  bool hil_connected,
                                                  bool ekf_init)
    {
        if (!connected_)
            return;

        // Timestamp (us -> ns)
        const uint64_t stamp_ns = fs.timestamp_us * 1000ULL;
        const bool publish_status =
            (last_status_pub_us_ == 0) ||
            (fs.timestamp_us < last_status_pub_us_) ||
            (fs.timestamp_us - last_status_pub_us_ >= STATUS_PERIOD_US);
        const bool publish_odom_tf =
            (last_odom_tf_pub_us_ == 0) ||
            (fs.timestamp_us < last_odom_tf_pub_us_) ||
            (fs.timestamp_us - last_odom_tf_pub_us_ >= ODOM_TF_PERIOD_US);
        const bool publish_truth_state =
            publish_truth_state_ &&
            (fs.truth_valid != 0) &&
            ((last_truth_state_pub_us_ == 0) ||
             (fs.timestamp_us < last_truth_state_pub_us_) ||
             (fs.timestamp_us - last_truth_state_pub_us_ >= TRUTH_STATE_PERIOD_US));

        // ── VehicleLocalPosition ──────────────────────────────────────────────
        {
            buffer_topic(
                out_best_effort_,
                dw_local_pos_id_,
                "VehicleLocalPosition",
                [&](dds_cdr::Writer &cdr)
                {
                    cdr.header(stamp_ns, "map_ned");
                    for (double value : fs.eta)
                        cdr.float64(value);
                    for (double value : fs.nu)
                        cdr.float64(value);
                    cdr.float64(fs.depth_m);
                    cdr.boolean(fs.dvl_valid != 0);
                    cdr.float64(
                        static_cast<double>(fs.timestamp_us) * 1e-6);
                });
        }

        // ── SensorCombined ────────────────────────────────────────────────────
        {
            const std::string sensor_frame = vehicle_ + "/base_link_frd";
            const bool gps_ok = (fs.gps_fix >= 3);
            buffer_topic(
                out_best_effort_,
                dw_sensor_id_,
                "SensorCombined",
                [&](dds_cdr::Writer &cdr)
                {
                    cdr.header(stamp_ns, sensor_frame.c_str());
                    for (float value : fs.acc)
                        cdr.float64(static_cast<double>(value));
                    for (float value : fs.gyro)
                        cdr.float64(static_cast<double>(value));
                    cdr.float64(fs.depth_m);
                    cdr.boolean(fs.dvl_valid != 0);
                    for (float value : fs.dvl_vel)
                        cdr.float64(static_cast<double>(value));
                    cdr.boolean(gps_ok);
                    cdr.int32(fs.gps_lat);
                    cdr.int32(fs.gps_lon);
                    cdr.int32(fs.gps_alt);
                    cdr.float64(fs.gps_vn);
                    cdr.float64(fs.gps_ve);
                    cdr.float64(fs.gps_vd);
                    cdr.uint8(fs.gps_satellites);
                });
        }

        // ── ActuatorOutputs ───────────────────────────────────────────────────
        {
            const std::string actuator_frame = vehicle_ + "/base_link_frd";
            buffer_topic(
                out_best_effort_,
                dw_actuator_id_,
                "ActuatorOutputs",
                [&](dds_cdr::Writer &cdr)
                {
                    cdr.header(stamp_ns, actuator_frame.c_str());
                    cdr.float32_array(fs.fins, 4);
                    cdr.float32(fs.thrust);
                    cdr.float64_array(fs.fin_deg, 4);
                    cdr.float64(static_cast<double>(fs.rpm));
                });
        }

        // ── VehicleStatus (1 Hz) ────────────────────────────────────
        if (publish_status)
        {
            const bool buffered = buffer_topic(
                out_reliable_,
                dw_status_id_,
                "VehicleStatus",
                [&](dds_cdr::Writer &cdr)
                {
                    cdr.header(stamp_ns);
                    cdr.string(gnc_mode_str.c_str());
                    cdr.uint8(hil_connected ? 1u : 0u);
                    cdr.boolean(hil_connected);
                    cdr.boolean(ekf_init);
                });
            if (buffered)
                last_status_pub_us_ = fs.timestamp_us;
        }

        // ── AUVState ─────────────────────────────────────────────────────────
        {
            buffer_topic(
                out_best_effort_,
                dw_auv_state_id_,
                "AUVState",
                [&](dds_cdr::Writer &cdr)
                {
                    cdr.header(stamp_ns, "map_ned");
                    cdr.string(vehicle_.c_str());
                    for (double value : fs.eta)
                        cdr.float64(value);
                    for (double value : fs.nu)
                        cdr.float64(value);
                    cdr.float64(fs.depth_m);
                    cdr.boolean(fs.dvl_valid != 0);
                    cdr.string(fs.mission_state);
                });
        }

        // ── TruthState (simulator/Fossen truth, debug only) ─────────────────────
        if (publish_truth_state)
        {
            const bool buffered = buffer_topic(
                out_best_effort_,
                dw_truth_state_id_,
                "TruthState",
                [&](dds_cdr::Writer &cdr)
                {
                    cdr.header(stamp_ns, "map_ned");
                    cdr.string(vehicle_.c_str());
                    for (double value : fs.truth_eta)
                        cdr.float64(value);
                    for (double value : fs.truth_nu)
                        cdr.float64(value);
                    cdr.float64(fs.truth_eta[2]);
                    cdr.boolean(true);
                    cdr.string("TRUTH");
                });
            if (buffered)
                last_truth_state_pub_us_ = fs.timestamp_us;
        }

        if (publish_odom_tf)
        {
            const std::string child_frame = vehicle_ + "/base_link_frd";

            const bool odom_buffered = buffer_topic(
                out_best_effort_,
                dw_odom_id_,
                "Odometry",
                [&](dds_cdr::Writer &cdr)
                {
                    cdr.header(stamp_ns, "map_ned");
                    cdr.string(child_frame.c_str());
                    cdr.float64(fs.eta[0]);
                    cdr.float64(fs.eta[1]);
                    cdr.float64(fs.eta[2]);
                    cdr.quaternion_rpy(fs.eta[3], fs.eta[4], fs.eta[5]);
                    cdr.covariance_36(fs.pose_cov);
                    for (double value : fs.nu)
                        cdr.float64(value);
                    cdr.covariance_36(fs.twist_cov);
                });

            const bool tf_buffered = buffer_topic(
                out_best_effort_,
                dw_tf_id_,
                "TFMessage",
                [&](dds_cdr::Writer &cdr)
                {
                    cdr.uint32(1u);
                    cdr.header(stamp_ns, "map_ned");
                    cdr.string(child_frame.c_str());
                    cdr.float64(fs.eta[0]);
                    cdr.float64(fs.eta[1]);
                    cdr.float64(fs.eta[2]);
                    cdr.quaternion_rpy(fs.eta[3], fs.eta[4], fs.eta[5]);
                });

            if (odom_buffered && tf_buffered)
                last_odom_tf_pub_us_ = fs.timestamp_us;
        }

        uxr_flash_output_streams(&session_);
    }


} // namespace hydrox

#endif // HYDROX_DDS_ENABLED
