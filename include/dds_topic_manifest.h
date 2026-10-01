#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace hydrox::dds_topics
{

enum class TopicScope : uint8_t
{
    HydroxVehicle,
    OceanXVehicle,
    Global,
};

enum class TopicFlow : uint8_t
{
    Publish,
    Subscribe,
};

struct TopicSpec
{
    const char *label;
    TopicScope scope;
    TopicFlow flow;
    const char *ros_suffix_or_name;
    const char *dds_suffix_or_name;
    const char *type_name;
    uint16_t topic_id;
    uint16_t endpoint_id;
    uint32_t rate_hz; // 0 means GNC tick or event-driven.
    const char *rate_note;
};

inline constexpr TopicSpec kVehicleLocalPosition{
    "vehicle_local_position",
    TopicScope::HydroxVehicle,
    TopicFlow::Publish,
    "/out/vehicle_local_position",
    "/out/vehicle_local_position",
    "oceanx_interfaces::msg::dds_::VehicleLocalPosition_",
    1,
    1,
    5,
    "5 Hz diagnostics",
};

inline constexpr TopicSpec kSensorCombined{
    "sensor_combined",
    TopicScope::HydroxVehicle,
    TopicFlow::Publish,
    "/out/sensor_combined",
    "/out/sensor_combined",
    "oceanx_interfaces::msg::dds_::SensorCombined_",
    2,
    2,
    5,
    "5 Hz diagnostics",
};

inline constexpr TopicSpec kActuatorOutputs{
    "actuator_outputs",
    TopicScope::HydroxVehicle,
    TopicFlow::Publish,
    "/out/actuator_outputs",
    "/out/actuator_outputs",
    "oceanx_interfaces::msg::dds_::ActuatorOutputs_",
    3,
    3,
    5,
    "5 Hz diagnostics",
};

inline constexpr TopicSpec kVehicleStatus{
    "vehicle_status",
    TopicScope::HydroxVehicle,
    TopicFlow::Publish,
    "/out/vehicle_status",
    "/out/vehicle_status",
    "oceanx_interfaces::msg::dds_::VehicleStatus_",
    4,
    4,
    1,
    "1 Hz",
};

inline constexpr TopicSpec kStateEstimate{
    "state_estimate",
    TopicScope::HydroxVehicle,
    TopicFlow::Publish,
    "/out/state_estimate",
    "/out/state_estimate",
    "oceanx_interfaces::msg::dds_::VehicleState_",
    5,
    5,
    20,
    "20 Hz control-state telemetry",
};

inline constexpr TopicSpec kTruthState{
    "truth_state",
    TopicScope::HydroxVehicle,
    TopicFlow::Publish,
    "/out/truth_state",
    "/out/truth_state",
    "oceanx_interfaces::msg::dds_::VehicleState_",
    9,
    9,
    5,
    "5 Hz debug when HIL truth is available",
};

inline constexpr TopicSpec kOdometry{
    "odom",
    TopicScope::HydroxVehicle,
    TopicFlow::Publish,
    "/out/odom",
    "/out/odom",
    "nav_msgs::msg::dds_::Odometry_",
    6,
    6,
    5,
    "5 Hz",
};

inline constexpr TopicSpec kTf{
    "tf",
    TopicScope::Global,
    TopicFlow::Publish,
    "/tf",
    "rt/tf",
    "tf2_msgs::msg::dds_::TFMessage_",
    7,
    7,
    5,
    "5 Hz",
};

inline constexpr TopicSpec kSetpoint{
    "setpoint",
    TopicScope::HydroxVehicle,
    TopicFlow::Subscribe,
    "/in/setpoint",
    "/in/setpoint",
    "oceanx_interfaces::msg::dds_::GNCSetpoint_",
    8,
    1,
    0,
    "event",
};

inline constexpr TopicSpec kPassiveSonarBearing{
    "passive_sonar_bearing",
    TopicScope::OceanXVehicle,
    TopicFlow::Publish,
    "/out/passive_sonar_bearing",
    "/out/passive_sonar_bearing",
    "oceanx_interfaces::msg::dds_::PassiveSonarBearing_",
    10,
    10,
    10,
    "sensor sample",
};

inline constexpr TopicSpec kAcousticNeighbors{
    "acoustic_neighbors",
    TopicScope::OceanXVehicle,
    TopicFlow::Publish,
    "/out/acoustic_neighbors",
    "/out/acoustic_neighbors",
    "oceanx_interfaces::msg::dds_::AcousticNeighbors_",
    11,
    11,
    10,
    "sensor sample",
};

inline constexpr TopicSpec kRangeFinderScan{
    "rangefinder_scan",
    TopicScope::OceanXVehicle,
    TopicFlow::Publish,
    "/out/rangefinder_scan",
    "/out/rangefinder_scan",
    "oceanx_interfaces::msg::dds_::RangeFinderScan_",
    12,
    12,
    10,
    "sensor sample",
};

inline constexpr std::array<TopicSpec, 12> kManifest{
    kVehicleLocalPosition,
    kSensorCombined,
    kActuatorOutputs,
    kVehicleStatus,
    kStateEstimate,
    kTruthState,
    kOdometry,
    kTf,
    kSetpoint,
    kPassiveSonarBearing,
    kAcousticNeighbors,
    kRangeFinderScan,
};

constexpr uint64_t period_us(const TopicSpec &topic)
{
    return topic.rate_hz > 0 ? (1'000'000ULL / topic.rate_hz) : 0ULL;
}

inline std::string dds_name(const TopicSpec &topic, const std::string &vehicle)
{
    if (topic.scope == TopicScope::Global)
        return topic.dds_suffix_or_name;
    if (topic.scope == TopicScope::OceanXVehicle)
        return "rt/oceanx/" + vehicle + topic.dds_suffix_or_name;
    return "rt/hydrox/" + vehicle + topic.dds_suffix_or_name;
}

inline std::string ros_name(const TopicSpec &topic, const std::string &vehicle)
{
    if (topic.scope == TopicScope::Global)
        return topic.ros_suffix_or_name;
    if (topic.scope == TopicScope::OceanXVehicle)
        return "/oceanx/" + vehicle + topic.ros_suffix_or_name;
    return "/hydrox/" + vehicle + topic.ros_suffix_or_name;
}

} // namespace hydrox::dds_topics
