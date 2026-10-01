#pragma once

// XLog 1.0 reader.
//
// The reader is intentionally independent from the recorder and from any
// transport.  It validates the file preamble, block headers and payload CRCs,
// skips a damaged block when possible, and treats an incomplete final block as
// a normal crash tail.  Callbacks are invoked synchronously while the payload
// is still owned by the reader; consumers must copy it if they need to retain
// it after returning from the callback.

#include "xlog_writer.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

namespace hydrox::xlog
{
  struct ReaderInternals;

  struct ReaderOptions
  {
    // Strict mode turns a damaged block into a read failure. Footer and
    // sequence requirements remain explicit because a crash-recovered log
    // normally has neither.
    bool strict = false;
    bool require_footer = false;
    bool require_contiguous_sequence = false;
    uint64_t max_preamble_bytes = 64ULL * 1024ULL * 1024ULL;
    // Refuse implausibly large blocks before allocating their payload buffer.
    uint32_t max_block_payload_bytes = 64U * 1024U * 1024U;
  };

  struct ReaderStats
  {
    uint64_t segment_count = 0;
    uint64_t block_count = 0;
    uint64_t record_count = 0;
    uint64_t skipped_block_count = 0;
    uint64_t skipped_record_count = 0;
    uint64_t corrupt_block_count = 0;
    uint64_t sequence_gap_count = 0;
    uint64_t truncated_tail_bytes = 0;
    bool footer_present = false;
    bool footer_valid = false;
  };

  struct RecordView
  {
    uint32_t segment_index = 0;
    uint64_t file_id_hi = 0;
    uint64_t file_id_lo = 0;
    RecordHeader header{};
    const uint8_t * payload = nullptr;
  };

  using RecordCallback = std::function<bool(const RecordView &)>;

  class Reader final
  {
  public:
    Reader() = default;

    Reader(const Reader &) = delete;
    Reader & operator=(const Reader &) = delete;

    // Reads path and any following _partNNNN segments emitted by Writer.
    // Returns false only for an invalid/unreadable file, strict-mode recovery
    // failure, or a callback that returns false.
    bool read(const std::string & path,
              const RecordCallback & callback,
              std::string * error = nullptr,
              const ReaderOptions & options = {});

    const std::string & metadata_json() const { return _metadata_json; }
    const std::string & schema_json() const { return _schema_json; }
    uint64_t file_id_hi() const { return _file_id_hi; }
    uint64_t file_id_lo() const { return _file_id_lo; }
    const ReaderStats & stats() const { return _stats; }

  private:
    friend struct ReaderInternals;

    std::string _metadata_json;
    std::string _schema_json;
    uint64_t _file_id_hi = 0;
    uint64_t _file_id_lo = 0;
    ReaderStats _stats{};
  };
} // namespace hydrox::xlog
