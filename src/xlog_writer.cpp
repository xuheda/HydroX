#include "xlog_writer.h"

#include <chrono>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace hydrox::xlog
{
namespace
{
constexpr uint64_t kFnvOffsetBasis = 14695981039346656037ULL;
constexpr uint64_t kFnvPrime = 1099511628211ULL;

bool is_little_endian()
{
  const uint16_t value = 1;
  return *reinterpret_cast<const uint8_t *>(&value) == 1;
}
}     // namespace

uint64_t unix_time_ns_now()
{
  return static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch())
    .count());
}

uint32_t crc32(const void * data, std::size_t size, uint32_t seed)
{
  uint32_t crc = ~seed;
  const auto * bytes = static_cast<const uint8_t *>(data);
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= bytes[i];
    for (int bit = 0; bit < 8; ++bit) {
      const uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }
  return ~crc;
}

uint64_t fnv1a64(const void * data, std::size_t size)
{
  uint64_t hash = kFnvOffsetBasis;
  const auto * bytes = static_cast<const uint8_t *>(data);
  for (std::size_t i = 0; i < size; ++i) {
    hash ^= bytes[i];
    hash *= kFnvPrime;
  }
  return hash;
}

std::string json_escape(const std::string & s)
{
  std::ostringstream out;
  for (const unsigned char ch : s) {
    switch (ch) {
      case '\\':
        out << "\\\\";
        break;
      case '"':
        out << "\\\"";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        if (ch < 0x20) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(ch) << std::dec;
        } else {
          out << ch;
        }
        break;
    }
  }
  return out.str();
}

std::string default_schema_json()
{
  return
    R"XLOG({
  "format":"XLog",
  "version":"1.0",
  "binary":{"endianness":"little","packing":1,"float":"IEEE-754","time_unit":"ns"},
  "topics":[
    {"id":1,"name":"/hydrox/state","type":"HydroxStateRecord","schema_version":1,"payload_size":112,"rate":"control tick (default 100 Hz)","fields":[
      {"name":"eta","type":"f64","count":6,"units":["m","m","m","rad","rad","rad"],"frame":"NED"},
      {"name":"nu","type":"f64","count":6,"units":["m/s","m/s","m/s","rad/s","rad/s","rad/s"],"frame":"FRD"},
      {"name":"depth_m","type":"f64","unit":"m"},{"name":"gnc_mode","type":"u8"},{"name":"mission_state","type":"u8"},
      {"name":"dvl_valid","type":"u8"},{"name":"ekf_init","type":"u8"},{"name":"reserved","type":"u8","count":4}]},
    {"id":2,"name":"/hydrox/setpoint","type":"HydroxSetpointRecord","schema_version":1,"payload_size":72,"rate":"on change + 1 Hz heartbeat","fields":[
      {"name":"depth_ref","type":"f64","unit":"m"},{"name":"heading_ref","type":"f64","unit":"rad"},
      {"name":"surge_ref","type":"f64","unit":"m/s"},{"name":"yaw_rate_ref","type":"f64","unit":"rad/s"},
      {"name":"wp_n","type":"f64","unit":"m"},{"name":"wp_e","type":"f64","unit":"m"},{"name":"wp_d","type":"f64","unit":"m"},
      {"name":"setpoint_age_s","type":"f64","unit":"s"},{"name":"use_yaw_rate_ref","type":"u8"},{"name":"reserved","type":"u8","count":7}]},
    {"id":3,"name":"/hydrox/control_error","type":"HydroxControlErrorRecord","schema_version":1,"payload_size":40,"rate":"control tick (default 100 Hz)","fields":[
      {"name":"depth_err","type":"f64","unit":"m"},{"name":"heading_err","type":"f64","unit":"rad"},
      {"name":"surge_err","type":"f64","unit":"m/s"},{"name":"yaw_rate_err","type":"f64","unit":"rad/s"},{"name":"wp_dist","type":"f64","unit":"m"}]},
    {"id":4,"name":"/hydrox/controller_output","type":"HydroxControllerOutputRecord","schema_version":1,"payload_size":56,"rate":"control tick (default 100 Hz)","fields":[
      {"name":"tau","type":"f64","count":6,"units":["N","N","N","N*m","N*m","N*m"],"frame":"FRD"},{"name":"tau_norm","type":"f64"}]},
    {"id":5,"name":"/hydrox/actuator","type":"HydroxActuatorRecord","schema_version":1,"payload_size":72,"rate":"control tick (default 100 Hz)","fields":[
      {"name":"ch","type":"f32","count":8,"unit":"normalized"},{"name":"rpm","type":"f64","unit":"rpm"},{"name":"thrust","type":"f64","unit":"normalized"},
      {"name":"max_abs_actuator","type":"f64","unit":"normalized"},{"name":"actuator_sat_ratio","type":"f64","unit":"ratio"},
      {"name":"active_count","type":"u8"},{"name":"reserved","type":"u8","count":7}]},
    {"id":6,"name":"/hydrox/estimator_health","type":"HydroxEstimatorHealthRecord","schema_version":1,"payload_size":48,"rate":"min(control rate, approximately 20 Hz)","fields":[
      {"name":"dvl_age_s","type":"f64","unit":"s"},{"name":"accel_norm","type":"f64","unit":"m/s^2"},{"name":"gyro_norm","type":"f64","unit":"rad/s"},
      {"name":"pose_cov_trace","type":"f64"},{"name":"twist_cov_trace","type":"f64"},{"name":"ekf_have_accel","type":"u8"},
      {"name":"dvl_valid","type":"u8"},{"name":"gps_valid","type":"u8"},{"name":"reserved","type":"u8","count":5}]},
    {"id":7,"name":"/hydrox/timing","type":"HydroxTimingRecord","schema_version":1,"payload_size":32,"rate":"control tick (default 100 Hz)","fields":[
      {"name":"dt","type":"f64","unit":"s"},{"name":"expected_dt","type":"f64","unit":"s"},{"name":"setpoint_age_s","type":"f64","unit":"s"},
      {"name":"loop_overrun","type":"u8"},{"name":"reserved","type":"u8","count":7}]},
    {"id":8,"name":"/simulator/truth","type":"SimulatorTruthRecord","schema_version":1,"payload_size":112,"rate":"min(control rate, approximately 20 Hz) when available","fields":[
      {"name":"eta","type":"f64","count":6,"units":["m","m","m","rad","rad","rad"],"frame":"NED"},
      {"name":"nu","type":"f64","count":6,"units":["m/s","m/s","m/s","rad/s","rad/s","rad/s"],"frame":"FRD"},
      {"name":"source_timestamp_us","type":"u64","unit":"us"},{"name":"valid","type":"u8"},{"name":"reserved","type":"u8","count":7}]},
    {"id":101,"name":"/deployment/alignment","type":"DeploymentAlignmentRecord","schema_version":1,"payload_size":528,"rate":"logger control tick","fields":[
      {"name":"tick","type":"u32"},{"name":"num_pursuers","type":"u32"},{"name":"dt_s","type":"f64","unit":"s"},{"name":"measured_dt_s","type":"f64","unit":"s"},
      {"name":"ue_n","type":"f64","count":4,"unit":"m"},{"name":"ue_e_rl","type":"f64","count":4,"unit":"m"},{"name":"ue_yaw_rl","type":"f64","count":4,"unit":"rad"},{"name":"ue_speed","type":"f64","count":4,"unit":"m/s"},
      {"name":"shadow_n","type":"f64","count":4,"unit":"m"},{"name":"shadow_e_rl","type":"f64","count":4,"unit":"m"},{"name":"shadow_yaw_rl","type":"f64","count":4,"unit":"rad"},{"name":"shadow_speed","type":"f64","count":4,"unit":"m/s"},
      {"name":"cmd_speed","type":"f64","count":4,"unit":"m/s"},{"name":"cmd_yaw_rate_rl","type":"f64","count":4,"unit":"rad/s"},
      {"name":"one_step_pos_err","type":"f64","count":4,"unit":"m"},{"name":"one_step_yaw_err","type":"f64","count":4,"unit":"rad"},
      {"name":"ue_target_n","type":"f64","unit":"m"},{"name":"ue_target_e_rl","type":"f64","unit":"m"},{"name":"ue_target_yaw_rl","type":"f64","unit":"rad"},{"name":"ue_target_speed","type":"f64","unit":"m/s"},
      {"name":"shadow_target_n","type":"f64","unit":"m"},{"name":"shadow_target_e_rl","type":"f64","unit":"m"},{"name":"shadow_target_yaw_rl","type":"f64","unit":"rad"},{"name":"shadow_target_speed","type":"f64","unit":"m/s"},
      {"name":"target_cmd_speed","type":"f64","unit":"m/s"},{"name":"target_cmd_yaw_rate_rl","type":"f64","unit":"rad/s"},
      {"name":"target_one_step_pos_err","type":"f64","unit":"m"},{"name":"target_one_step_yaw_err","type":"f64","unit":"rad"},
      {"name":"ue_min_dist","type":"f64","unit":"m"},{"name":"shadow_min_dist","type":"f64","unit":"m"},
      {"name":"shadow_initialized","type":"u8"},{"name":"reserved","type":"u8","count":7}]}
  ]
})XLOG";
}

Writer::~Writer()
{
  close();
}

bool Writer::open(
  const std::string & path,
  const std::string & metadata_json,
  std::string * error,
  const WriterOptions & options)
{
  close();
  _failed = false;
  _last_error.clear();
  _base_path = path;
  _path.clear();
  _metadata_json = metadata_json;
  _schema_json = default_schema_json();
  _options = options;
  _segment_paths.clear();
  _block_payload.clear();
  _sequence = 0;
  _segment_index = 0;

  if (!is_little_endian()) {
    mark_failed("XLog 1.0 writer requires a little-endian host");
  } else if (_base_path.empty()) {
    mark_failed("XLog path is empty");
  } else if (_options.target_block_bytes < sizeof(RecordHeader) + 1U) {
    mark_failed("XLog target_block_bytes is too small");
  }

  if (_failed) {
    if (error) {
      *error = _last_error;
    }
    return false;
  }

  _start_unix_ns = unix_time_ns_now();
  _schema_hash = fnv1a64(_schema_json.data(), _schema_json.size());
  _file_id_hi = _start_unix_ns;
  _file_id_lo = fnv1a64(_metadata_json.data(), _metadata_json.size()) ^
    _schema_hash ^ (_start_unix_ns * 0x9E3779B185EBCA87ULL);

  if (!open_segment(0, error)) {
    return false;
  }

  return true;
}

bool Writer::open_segment(uint32_t segment_index, std::string * error)
{
  _segment_index = segment_index;
  _path = make_segment_path(segment_index);

  try {
    const std::filesystem::path p(_path);
    const auto parent = p.parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent);
    }
  } catch (const std::exception & e) {
    mark_failed(std::string("failed to create XLog directory: ") + e.what());
  }

  if (!_failed) {
    _out.clear();
    _out.open(_path, std::ios::binary | std::ios::out | std::ios::trunc);
    if (!_out) {
      mark_failed("failed to open XLog segment: " + _path);
    }
  }

  if (_failed) {
    if (error) {
      *error = _last_error;
    }
    return false;
  }

  FileHeader header;
  header.header_size = sizeof(FileHeader);
  header.record_header_size = sizeof(RecordHeader);
  header.block_header_size = sizeof(BlockHeader);
  header.segment_index = segment_index;
  header.start_unix_ns = _start_unix_ns;
  header.metadata_len = static_cast<uint64_t>(_metadata_json.size());
  header.schema_len = static_cast<uint64_t>(_schema_json.size());
  header.schema_hash = _schema_hash;
  header.max_segment_bytes = _options.max_segment_bytes;
  header.target_block_bytes = _options.target_block_bytes;
  header.file_id_hi = _file_id_hi;
  header.file_id_lo = _file_id_lo;

  FileHeader crc_header = header;
  crc_header.header_crc32 = 0;
  uint32_t header_crc = crc32(&crc_header, sizeof(crc_header));
  header_crc = crc32(_metadata_json.data(), _metadata_json.size(), header_crc);
  header_crc = crc32(_schema_json.data(), _schema_json.size(), header_crc);
  header.header_crc32 = header_crc;

  _out.write(reinterpret_cast<const char *>(&header), sizeof(header));
  _out.write(_metadata_json.data(), static_cast<std::streamsize>(_metadata_json.size()));
  _out.write(_schema_json.data(), static_cast<std::streamsize>(_schema_json.size()));
  _out.flush();
  if (!_out.good()) {
    mark_failed("failed to write XLog segment header: " + _path);
  }

  if (_failed) {
    _out.close();
    if (error) {
      *error = _last_error;
    }
    return false;
  }

  _segment_paths.push_back(_path);
  _segment_bytes = sizeof(FileHeader) + _metadata_json.size() + _schema_json.size();
  _segment_block_count = 0;
  _segment_record_count = 0;
  _segment_first_timestamp_ns = 0;
  _segment_last_timestamp_ns = 0;
  _segment_last_sequence = 0;
  return true;
}

bool Writer::append_block_bytes(const void * data, std::size_t size)
{
  try {
    const auto * bytes = static_cast<const uint8_t *>(data);
    _block_payload.insert(_block_payload.end(), bytes, bytes + size);
    return true;
  } catch (const std::exception & e) {
    mark_failed(std::string("failed to buffer XLog record: ") + e.what());
    return false;
  }
}

bool Writer::write(
  TopicId topic,
  uint64_t timestamp_ns,
  const void * payload,
  uint32_t payload_size)
{
  if (!is_open()) {
    return false;
  }
  if (payload_size > 0 && payload == nullptr) {
    mark_failed("XLog record payload is null");
    return false;
  }

  const uint64_t record_bytes = sizeof(RecordHeader) + static_cast<uint64_t>(payload_size);
  if (_block_record_count > 0 &&
    _block_payload.size() + record_bytes > _options.target_block_bytes)
  {
    if (!flush_block()) {
      return false;
    }
  }

  RecordHeader header;
  header.topic_id = static_cast<uint16_t>(topic);
  header.payload_size = payload_size;
  header.timestamp_ns = timestamp_ns;
  header.sequence = _sequence;

  if (!append_block_bytes(&header, sizeof(header)) ||
    (payload_size > 0 && !append_block_bytes(payload, payload_size)))
  {
    return false;
  }

  if (_block_record_count == 0) {
    _block_first_timestamp_ns = timestamp_ns;
    _block_first_sequence = _sequence;
  }
  _block_last_timestamp_ns = timestamp_ns;
  _block_last_sequence = _sequence;
  ++_block_record_count;
  ++_sequence;

  if (_block_payload.size() >= _options.target_block_bytes) {
    return flush_block();
  }
  return true;
}

bool Writer::flush_block()
{
  if (_block_record_count == 0) {
    return !_failed;
  }
  if (!is_open()) {
    return false;
  }

  const uint64_t required_bytes = sizeof(BlockHeader) +
    static_cast<uint64_t>(_block_payload.size()) +
    sizeof(FileFooter);
  if (_options.max_segment_bytes > 0 &&
    _segment_block_count > 0 &&
    _segment_bytes + required_bytes > _options.max_segment_bytes)
  {
    if (!finalize_segment()) {
      return false;
    }
    std::string error;
    if (!open_segment(_segment_index + 1U, &error)) {
      return false;
    }
  }

  BlockHeader header;
  header.header_size = sizeof(BlockHeader);
  header.payload_size = static_cast<uint32_t>(_block_payload.size());
  header.record_count = _block_record_count;
  header.payload_crc32 = crc32(_block_payload.data(), _block_payload.size());
  header.first_timestamp_ns = _block_first_timestamp_ns;
  header.last_timestamp_ns = _block_last_timestamp_ns;
  header.first_sequence = _block_first_sequence;
  header.last_sequence = _block_last_sequence;

  BlockHeader crc_header = header;
  crc_header.header_crc32 = 0;
  header.header_crc32 = crc32(&crc_header, sizeof(crc_header));

  _out.write(reinterpret_cast<const char *>(&header), sizeof(header));
  _out.write(
    reinterpret_cast<const char *>(_block_payload.data()),
    static_cast<std::streamsize>(_block_payload.size()));
  if (!_out.good()) {
    mark_failed("failed to write XLog block: " + _path);
    return false;
  }

  if (_segment_block_count == 0) {
    _segment_first_timestamp_ns = _block_first_timestamp_ns;
  }
  _segment_last_timestamp_ns = _block_last_timestamp_ns;
  _segment_last_sequence = _block_last_sequence;
  ++_segment_block_count;
  _segment_record_count += _block_record_count;
  _segment_bytes += sizeof(BlockHeader) + _block_payload.size();

  _block_payload.clear();
  _block_record_count = 0;
  _block_first_timestamp_ns = 0;
  _block_last_timestamp_ns = 0;
  _block_first_sequence = 0;
  _block_last_sequence = 0;
  return true;
}

bool Writer::finalize_segment()
{
  if (!_out.is_open()) {
    return !_failed;
  }
  if (_failed) {
    _out.close();
    return false;
  }

  FileFooter footer;
  footer.footer_size = sizeof(FileFooter);
  footer.block_count = _segment_block_count;
  footer.record_count = _segment_record_count;
  footer.first_timestamp_ns = _segment_first_timestamp_ns;
  footer.last_timestamp_ns = _segment_last_timestamp_ns;
  footer.file_bytes = _segment_bytes + sizeof(FileFooter);
  footer.next_sequence = _segment_record_count > 0 ?
    _segment_last_sequence + 1ULL :
    _sequence;

  FileFooter crc_footer = footer;
  crc_footer.footer_crc32 = 0;
  footer.footer_crc32 = crc32(&crc_footer, sizeof(crc_footer));

  _out.write(reinterpret_cast<const char *>(&footer), sizeof(footer));
  _out.flush();
  if (!_out.good()) {
    mark_failed("failed to finalize XLog segment: " + _path);
    _out.close();
    return false;
  }

  _segment_bytes += sizeof(FileFooter);
  _out.close();
  return true;
}

bool Writer::flush()
{
  if (!is_open()) {
    return false;
  }
  if (!flush_block()) {
    return false;
  }
  _out.flush();
  if (!_out.good()) {
    mark_failed("failed to flush XLog segment: " + _path);
    return false;
  }
  return true;
}

void Writer::close()
{
  if (_out.is_open()) {
    if (!_failed && flush_block()) {
      finalize_segment();
    } else {
      _out.close();
    }
  }

  _block_payload.clear();
  _block_record_count = 0;
}

void Writer::mark_failed(const std::string & message)
{
  if (!_failed) {
    _last_error = message;
  }
  _failed = true;
}

std::string Writer::make_segment_path(uint32_t segment_index) const
{
  if (segment_index == 0) {
    return _base_path;
  }

  const std::filesystem::path base(_base_path);
  std::ostringstream filename;
  filename << base.stem().string()
           << "_part" << std::setw(4) << std::setfill('0') << (segment_index + 1U)
           << base.extension().string();
  return (base.parent_path() / filename.str()).string();
}

} // namespace hydrox::xlog
