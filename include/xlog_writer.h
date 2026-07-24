#pragma once

// XLog 1.0: chunked, checksummed OceanX binary time-series log.
//
// File layout:
//   FileHeader (96 bytes)
//   metadata JSON
//   schema JSON
//   repeated {BlockHeader (64 bytes) + block payload}
//   optional FileFooter (64 bytes, present after a clean close)
//
// A block payload is a sequence of {RecordHeader + packed payload}. Complete
// blocks remain readable after a crash; an incomplete tail is ignored. Files
// split at the configured segment size and retain one file id and sequence.

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace hydrox::xlog
{
  constexpr uint16_t kVersionMajor = 1;
  constexpr uint16_t kVersionMinor = 0;
  constexpr uint16_t kBlockVersion = 1;

  constexpr uint8_t kFileFlagChunked = 1u << 0;
  constexpr uint8_t kFileFlagSegmented = 1u << 1;
  constexpr uint8_t kFileFlagFooter = 1u << 2;

  constexpr uint64_t kDefaultMaxSegmentBytes = 512ULL * 1024ULL * 1024ULL;
  constexpr uint32_t kDefaultTargetBlockBytes = 1024U * 1024U;

  enum class TopicId : uint16_t
  {
    HydroxState = 1,
    HydroxSetpoint = 2,
    HydroxControlError = 3,
    HydroxControllerOutput = 4,
    HydroxActuator = 5,
    HydroxEstimatorHealth = 6,
    HydroxTiming = 7,
    SimulatorTruth = 8,
    DeploymentAlignment = 101,
  };

  struct WriterOptions
  {
    uint64_t max_segment_bytes = kDefaultMaxSegmentBytes;
    uint32_t target_block_bytes = kDefaultTargetBlockBytes;
  };

  constexpr std::size_t kDeploymentMaxPursuers = 4;

#pragma pack(push, 1)
  struct FileHeader
  {
    char magic[4] = {'X', 'L', 'O', 'G'};
    uint16_t version_major = kVersionMajor;
    uint16_t version_minor = kVersionMinor;
    uint16_t header_size = 0;
    uint8_t endian = 1;     // 1 = little endian
    uint8_t flags = kFileFlagChunked | kFileFlagSegmented | kFileFlagFooter;
    uint32_t record_header_size = 0;
    uint32_t block_header_size = 0;
    uint32_t segment_index = 0;
    uint32_t header_crc32 = 0;     // Header with this field zero + metadata + schema.
    uint32_t reserved0 = 0;
    uint64_t start_unix_ns = 0;
    uint64_t metadata_len = 0;
    uint64_t schema_len = 0;
    uint64_t schema_hash = 0;     // FNV-1a 64 over schema JSON.
    uint64_t max_segment_bytes = 0;
    uint64_t target_block_bytes = 0;
    uint64_t file_id_hi = 0;
    uint64_t file_id_lo = 0;
  };

  struct BlockHeader
  {
    char magic[4] = {'X', 'B', 'L', 'K'};
    uint16_t header_size = 0;
    uint16_t block_version = kBlockVersion;
    uint32_t flags = 0;
    uint32_t payload_size = 0;
    uint32_t record_count = 0;
    uint32_t payload_crc32 = 0;
    uint64_t first_timestamp_ns = 0;
    uint64_t last_timestamp_ns = 0;
    uint64_t first_sequence = 0;
    uint64_t last_sequence = 0;
    uint32_t header_crc32 = 0;     // Header with this field zero.
    uint32_t reserved = 0;
  };

  struct RecordHeader
  {
    uint16_t topic_id = 0;
    uint16_t flags = 0;
    uint32_t payload_size = 0;
    uint64_t timestamp_ns = 0;
    uint64_t sequence = 0;
  };

  struct FileFooter
  {
    char magic[4] = {'X', 'E', 'N', 'D'};
    uint16_t footer_size = 0;
    uint16_t footer_version = 1;
    uint32_t flags = 0;
    uint32_t block_count = 0;
    uint64_t record_count = 0;
    uint64_t first_timestamp_ns = 0;
    uint64_t last_timestamp_ns = 0;
    uint64_t file_bytes = 0;
    uint64_t next_sequence = 0;
    uint32_t footer_crc32 = 0;     // Footer with this field zero.
    uint32_t reserved = 0;
  };

  struct HydroxStateRecord
  {
    double eta[6] = {};
    double nu[6] = {};
    double depth_m = 0.0;
    uint8_t gnc_mode = 0;
    uint8_t mission_state = 0;     // 0 IDLE, 1 RUNNING, 2 COMPLETE, 3 FAILED
    uint8_t dvl_valid = 0;
    uint8_t ekf_init = 0;
    uint8_t reserved[4] = {};
  };

  struct HydroxSetpointRecord
  {
    double depth_ref = 0.0;
    double heading_ref = 0.0;
    double surge_ref = 0.0;
    double yaw_rate_ref = 0.0;
    double wp_n = 0.0;
    double wp_e = 0.0;
    double wp_d = 0.0;
    double setpoint_age_s = -1.0;
    uint8_t use_yaw_rate_ref = 0;
    uint8_t reserved[7] = {};
  };

  struct HydroxControlErrorRecord
  {
    double depth_err = 0.0;
    double heading_err = 0.0;
    double surge_err = 0.0;
    double yaw_rate_err = 0.0;
    double wp_dist = -1.0;
  };

  struct HydroxControllerOutputRecord
  {
    double tau[6] = {};
    double tau_norm = 0.0;
  };

  struct HydroxActuatorRecord
  {
    float ch[8] = {};
    double rpm = 0.0;
    double thrust = 0.0;
    double max_abs_actuator = 0.0;
    double actuator_sat_ratio = 0.0;
    uint8_t active_count = 0;
    uint8_t reserved[7] = {};
  };

  struct HydroxEstimatorHealthRecord
  {
    double dvl_age_s = -1.0;
    double accel_norm = 0.0;
    double gyro_norm = 0.0;
    double pose_cov_trace = 0.0;
    double twist_cov_trace = 0.0;
    uint8_t ekf_have_accel = 0;
    uint8_t dvl_valid = 0;
    uint8_t gps_valid = 0;
    uint8_t reserved[5] = {};
  };

  struct HydroxTimingRecord
  {
    double dt = 0.0;
    double expected_dt = 0.0;
    double setpoint_age_s = -1.0;
    uint8_t loop_overrun = 0;
    uint8_t reserved[7] = {};
  };

  struct SimulatorTruthRecord
  {
    double eta[6] = {};
    double nu[6] = {};
    uint64_t source_timestamp_us = 0;
    uint8_t valid = 0;
    uint8_t reserved[7] = {};
  };

  struct DeploymentAlignmentRecord
  {
    uint32_t tick = 0;
    uint32_t num_pursuers = 0;
    double dt_s = 0.0;
    double measured_dt_s = 0.0;

    double ue_n[kDeploymentMaxPursuers] = {};
    double ue_e_rl[kDeploymentMaxPursuers] = {};
    double ue_yaw_rl[kDeploymentMaxPursuers] = {};
    double ue_speed[kDeploymentMaxPursuers] = {};

    double shadow_n[kDeploymentMaxPursuers] = {};
    double shadow_e_rl[kDeploymentMaxPursuers] = {};
    double shadow_yaw_rl[kDeploymentMaxPursuers] = {};
    double shadow_speed[kDeploymentMaxPursuers] = {};

    double cmd_speed[kDeploymentMaxPursuers] = {};
    double cmd_yaw_rate_rl[kDeploymentMaxPursuers] = {};
    double one_step_pos_err[kDeploymentMaxPursuers] = {};
    double one_step_yaw_err[kDeploymentMaxPursuers] = {};

    double ue_target_n = 0.0;
    double ue_target_e_rl = 0.0;
    double ue_target_yaw_rl = 0.0;
    double ue_target_speed = 0.0;

    double shadow_target_n = 0.0;
    double shadow_target_e_rl = 0.0;
    double shadow_target_yaw_rl = 0.0;
    double shadow_target_speed = 0.0;

    double target_cmd_speed = 0.0;
    double target_cmd_yaw_rate_rl = 0.0;
    double target_one_step_pos_err = 0.0;
    double target_one_step_yaw_err = 0.0;
    double ue_min_dist = 0.0;
    double shadow_min_dist = 0.0;

    uint8_t shadow_initialized = 0;
    uint8_t reserved[7] = {};
  };
#pragma pack(pop)

  static_assert(sizeof(FileHeader) == 96, "XLog 1.0 FileHeader layout changed");
  static_assert(sizeof(BlockHeader) == 64, "XLog 1.0 BlockHeader layout changed");
  static_assert(sizeof(RecordHeader) == 24, "XLog 1.0 RecordHeader layout changed");
  static_assert(sizeof(FileFooter) == 64, "XLog 1.0 FileFooter layout changed");
  static_assert(sizeof(HydroxStateRecord) == 112, "HydroxStateRecord layout changed");
  static_assert(sizeof(HydroxSetpointRecord) == 72, "HydroxSetpointRecord layout changed");
  static_assert(sizeof(HydroxControlErrorRecord) == 40, "HydroxControlErrorRecord layout changed");
  static_assert(
    sizeof(HydroxControllerOutputRecord) == 56,
    "HydroxControllerOutputRecord layout changed");
  static_assert(sizeof(HydroxActuatorRecord) == 72, "HydroxActuatorRecord layout changed");
  static_assert(
    sizeof(HydroxEstimatorHealthRecord) == 48,
    "HydroxEstimatorHealthRecord layout changed");
  static_assert(sizeof(HydroxTimingRecord) == 32, "HydroxTimingRecord layout changed");
  static_assert(sizeof(SimulatorTruthRecord) == 112, "SimulatorTruthRecord layout changed");
  static_assert(
    sizeof(DeploymentAlignmentRecord) == 528,
    "DeploymentAlignmentRecord layout changed");

  class Writer
  {
public:
    Writer() = default;
    ~Writer();
    Writer(const Writer &) = delete;
    Writer & operator = (const Writer &) = delete;

    bool open(
      const std::string & path,
      const std::string & metadata_json,
      std::string * error = nullptr,
      const WriterOptions & options = {});
    bool is_open() const {return _out.is_open() && !_failed;}
    bool failed() const {return _failed;}
    const std::string & path() const {return _path;}
    const std::string & base_path() const {return _base_path;}
    const std::string & last_error() const {return _last_error;}
    const std::vector < std::string > & segment_paths() const {
      return _segment_paths;
    }
    void close();

    bool write(
      TopicId topic,
      uint64_t timestamp_ns,
      const void * payload,
      uint32_t payload_size);

    template < typename T >
    bool write(TopicId topic, uint64_t timestamp_ns, const T & payload)
    {
      return write(topic, timestamp_ns, &payload, static_cast < uint32_t > (sizeof(T)));
    }

    bool flush();

private:
    bool open_segment(uint32_t segment_index, std::string * error);
    bool flush_block();
    bool finalize_segment();
    bool append_block_bytes(const void * data, std::size_t size);
    void mark_failed(const std::string & message);
    std::string make_segment_path(uint32_t segment_index) const;

    std::ofstream _out;
    std::string _base_path;
    std::string _path;
    std::string _metadata_json;
    std::string _schema_json;
    std::string _last_error;
    WriterOptions _options;
    std::vector < std::string > _segment_paths;
    std::vector < uint8_t > _block_payload;

    uint64_t _start_unix_ns = 0;
    uint64_t _schema_hash = 0;
    uint64_t _file_id_hi = 0;
    uint64_t _file_id_lo = 0;
    uint64_t _sequence = 0;

    uint32_t _segment_index = 0;
    uint64_t _segment_bytes = 0;
    uint32_t _segment_block_count = 0;
    uint64_t _segment_record_count = 0;
    uint64_t _segment_first_timestamp_ns = 0;
    uint64_t _segment_last_timestamp_ns = 0;
    uint64_t _segment_last_sequence = 0;

    uint32_t _block_record_count = 0;
    uint64_t _block_first_timestamp_ns = 0;
    uint64_t _block_last_timestamp_ns = 0;
    uint64_t _block_first_sequence = 0;
    uint64_t _block_last_sequence = 0;
    bool _failed = false;
  };

  std::string default_schema_json();
  std::string json_escape(const std::string & s);
  uint64_t unix_time_ns_now();
  uint32_t crc32(const void * data, std::size_t size, uint32_t seed = 0);
  uint64_t fnv1a64(const void * data, std::size_t size);

} // namespace hydrox::xlog
