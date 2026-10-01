#include "xlog_reader.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>
#include <vector>

namespace hydrox::xlog
{
struct ReaderInternals
{
  static void set_preamble(Reader & reader,
                           std::string metadata,
                           std::string schema,
                           uint64_t file_id_hi,
                           uint64_t file_id_lo)
  {
    reader._metadata_json = std::move(metadata);
    reader._schema_json = std::move(schema);
    reader._file_id_hi = file_id_hi;
    reader._file_id_lo = file_id_lo;
  }

  static ReaderStats & stats(Reader & reader)
  {
    return reader._stats;
  }
};

namespace
{
  using Path = std::filesystem::path;

  bool read_exact(std::ifstream & input, void * data, std::size_t size)
  {
    if (size == 0)
      return true;
    input.read(static_cast<char *>(data), static_cast<std::streamsize>(size));
    return input.good() && static_cast<std::size_t>(input.gcount()) == size;
  }

  bool fits(uint64_t offset, uint64_t size, uint64_t file_size)
  {
    return offset <= file_size && size <= file_size - offset;
  }

  std::string segment_path(const Path & base, uint32_t index)
  {
    if (index == 0)
      return base.string();
    std::ostringstream name;
    name << base.stem().string() << "_part" << std::setw(4) << std::setfill('0')
         << (index + 1U) << base.extension().string();
    return (base.parent_path() / name.str()).string();
  }

  bool has_part_suffix(const Path & path)
  {
    const std::string stem = path.stem().string();
    if (stem.size() < 9 || stem[stem.size() - 9] != '_')
      return false;
    if (stem.compare(stem.size() - 8, 4, "part") != 0)
      return false;
    for (std::size_t i = stem.size() - 4; i < stem.size(); ++i)
      if (stem[i] < '0' || stem[i] > '9')
        return false;
    return true;
  }

  Path base_path_for(const Path & path)
  {
    if (!has_part_suffix(path))
      return path;
    std::string stem = path.stem().string();
    stem.resize(stem.size() - 9);
    return path.parent_path() / (stem + path.extension().string());
  }

  bool same_preamble(const FileHeader & header,
                     const std::string & metadata,
                     const std::string & schema,
                     const Reader & reader,
                     std::string * error)
  {
    if (header.file_id_hi != reader.file_id_hi() ||
        header.file_id_lo != reader.file_id_lo() ||
        metadata != reader.metadata_json() || schema != reader.schema_json())
    {
      if (error)
        *error = "XLog segment identity or schema does not match the first segment";
      return false;
    }
    return true;
  }

  bool find_magic(std::ifstream & input, uint64_t & offset, uint64_t file_size)
  {
    std::array<char, 4> magic{};
    for (uint64_t candidate = offset; fits(candidate, magic.size(), file_size); ++candidate)
    {
      input.clear();
      input.seekg(static_cast<std::streamoff>(candidate), std::ios::beg);
      if (!read_exact(input, magic.data(), magic.size()))
        return false;
      if (std::memcmp(magic.data(), "XBLK", 4) == 0 ||
          std::memcmp(magic.data(), "XEND", 4) == 0)
      {
        offset = candidate;
        return true;
      }
    }
    offset = file_size;
    return false;
  }

  bool read_segment(const Path & path,
                    uint32_t expected_segment,
                    Reader & reader,
                    const RecordCallback & callback,
                    const ReaderOptions & options,
                    uint64_t & previous_sequence,
                    bool & have_sequence,
                    std::string * error)
  {
    std::error_code size_error;
    const uintmax_t file_size_native = std::filesystem::file_size(path, size_error);
    if (size_error || file_size_native > std::numeric_limits<uint64_t>::max())
    {
      if (error) *error = "cannot stat XLog segment: " + path.string();
      return false;
    }
    const uint64_t file_size = static_cast<uint64_t>(file_size_native);
    std::ifstream input(path, std::ios::binary);
    if (!input)
    {
      if (error) *error = "cannot open XLog segment: " + path.string();
      return false;
    }

    FileHeader header{};
    if (!read_exact(input, &header, sizeof(header)))
    {
      if (error) *error = "truncated XLog file header: " + path.string();
      return false;
    }
    uint64_t offset = sizeof(header);
    if (std::memcmp(header.magic, "XLOG", 4) != 0 ||
        header.version_major != kVersionMajor ||
        header.version_minor != kVersionMinor ||
        header.endian != 1 || header.header_size != sizeof(FileHeader) ||
        header.record_header_size != sizeof(RecordHeader) ||
        header.block_header_size != sizeof(BlockHeader) ||
        header.segment_index != expected_segment ||
        header.metadata_len > std::numeric_limits<std::size_t>::max() ||
        header.schema_len > std::numeric_limits<std::size_t>::max() ||
        header.metadata_len > options.max_preamble_bytes ||
        header.schema_len > options.max_preamble_bytes - header.metadata_len ||
        !fits(offset, header.metadata_len, file_size) ||
        !fits(offset + header.metadata_len, header.schema_len, file_size))
    {
      if (error) *error = "invalid XLog file header: " + path.string();
      return false;
    }

    std::string metadata(static_cast<std::size_t>(header.metadata_len), '\0');
    std::string schema(static_cast<std::size_t>(header.schema_len), '\0');
    if (!read_exact(input, metadata.data(), metadata.size()) ||
        !read_exact(input, schema.data(), schema.size()))
    {
      if (error) *error = "truncated XLog metadata/schema: " + path.string();
      return false;
    }
    offset += header.metadata_len + header.schema_len;

    FileHeader header_for_crc = header;
    header_for_crc.header_crc32 = 0;
    uint32_t header_crc = crc32(&header_for_crc, sizeof(header_for_crc));
    header_crc = crc32(metadata.data(), metadata.size(), header_crc);
    header_crc = crc32(schema.data(), schema.size(), header_crc);
    if (header_crc != header.header_crc32 ||
        fnv1a64(schema.data(), schema.size()) != header.schema_hash)
    {
      if (error) *error = "XLog preamble checksum/schema hash mismatch: " + path.string();
      return false;
    }

    if (reader.stats().segment_count == 0)
    {
      ReaderInternals::set_preamble(
        reader, metadata, schema, header.file_id_hi, header.file_id_lo);
    }
    else if (!same_preamble(header, metadata, schema, reader, error))
    {
      return false;
    }

    bool segment_footer_present = false;
    bool segment_footer_valid = false;
    while (offset < file_size)
    {
      const uint64_t block_start = offset;
      if (!fits(offset, sizeof(uint32_t), file_size))
      {
        ReaderInternals::stats(reader).truncated_tail_bytes += file_size - offset;
        break;
      }

      std::array<char, 4> magic{};
      input.clear();
      input.seekg(static_cast<std::streamoff>(offset), std::ios::beg);
      if (!read_exact(input, magic.data(), magic.size()))
      {
        ReaderInternals::stats(reader).truncated_tail_bytes += file_size - offset;
        break;
      }

      if (std::memcmp(magic.data(), "XEND", 4) == 0)
      {
        if (!fits(offset, sizeof(FileFooter), file_size))
        {
          ReaderInternals::stats(reader).truncated_tail_bytes += file_size - offset;
          break;
        }
        FileFooter footer{};
        input.clear();
        input.seekg(static_cast<std::streamoff>(offset), std::ios::beg);
        if (!read_exact(input, &footer, sizeof(footer)))
        {
          ReaderInternals::stats(reader).truncated_tail_bytes += file_size - offset;
          break;
        }
        FileFooter footer_for_crc = footer;
        footer_for_crc.footer_crc32 = 0;
        const bool valid = footer.footer_size == sizeof(FileFooter) &&
                           footer.file_bytes == file_size &&
                           crc32(&footer_for_crc, sizeof(footer_for_crc)) ==
                             footer.footer_crc32;
        segment_footer_present = true;
        segment_footer_valid = valid;
        if (!valid && options.strict)
        {
          if (error) *error = "invalid XLog footer: " + path.string();
          return false;
        }
        offset += sizeof(FileFooter);
        if (offset != file_size)
        {
          ReaderInternals::stats(reader).truncated_tail_bytes += file_size - offset;
        }
        break;
      }

      if (std::memcmp(magic.data(), "XBLK", 4) != 0 ||
          !fits(offset, sizeof(BlockHeader), file_size))
      {
        ++ReaderInternals::stats(reader).corrupt_block_count;
        ++ReaderInternals::stats(reader).skipped_block_count;
        if (options.strict)
        {
          if (error) *error = "invalid XLog block marker/header: " + path.string();
          return false;
        }
        offset = block_start + 1;
        if (!find_magic(input, offset, file_size))
          break;
        continue;
      }

      BlockHeader block{};
      input.clear();
      input.seekg(static_cast<std::streamoff>(offset), std::ios::beg);
      if (!read_exact(input, &block, sizeof(block)))
      {
        ReaderInternals::stats(reader).truncated_tail_bytes += file_size - offset;
        break;
      }
      const bool header_valid = block.header_size == sizeof(BlockHeader) &&
                                block.block_version == kBlockVersion &&
                                block.record_count > 0 &&
                                block.payload_size <= options.max_block_payload_bytes;
      BlockHeader block_for_crc = block;
      block_for_crc.header_crc32 = 0;
      const bool header_crc_valid =
        crc32(&block_for_crc, sizeof(block_for_crc)) == block.header_crc32;
      if (!header_valid || !header_crc_valid)
      {
        ++ReaderInternals::stats(reader).corrupt_block_count;
        ++ReaderInternals::stats(reader).skipped_block_count;
        if (options.strict)
        {
          if (error) *error = "invalid XLog block header/CRC: " + path.string();
          return false;
        }
        offset = block_start + 1;
        if (!find_magic(input, offset, file_size))
          break;
        continue;
      }

      if (!fits(offset + sizeof(BlockHeader), block.payload_size, file_size))
      {
        ReaderInternals::stats(reader).truncated_tail_bytes += file_size - offset;
        break;
      }

      std::vector<uint8_t> payload(block.payload_size);
      if (!read_exact(input, payload.data(), payload.size()))
      {
        ReaderInternals::stats(reader).truncated_tail_bytes += file_size - offset;
        break;
      }
      offset += sizeof(BlockHeader) + block.payload_size;
      if (crc32(payload.data(), payload.size()) != block.payload_crc32)
      {
        ++ReaderInternals::stats(reader).corrupt_block_count;
        ++ReaderInternals::stats(reader).skipped_block_count;
        ReaderInternals::stats(reader).skipped_record_count += block.record_count;
        if (options.strict)
        {
          if (error) *error = "invalid XLog block payload CRC: " + path.string();
          return false;
        }
        continue;
      }

      std::size_t cursor = 0;
      std::vector<RecordHeader> records;
      std::vector<const uint8_t *> record_payloads;
      records.reserve(block.record_count);
      record_payloads.reserve(block.record_count);
      bool records_valid = true;
      for (uint32_t index = 0; index < block.record_count; ++index)
      {
        if (payload.size() - cursor < sizeof(RecordHeader))
        {
          records_valid = false;
          break;
        }
        RecordHeader record{};
        std::memcpy(&record, payload.data() + cursor, sizeof(record));
        cursor += sizeof(record);
        if (record.payload_size > payload.size() - cursor)
        {
          records_valid = false;
          break;
        }
        records.push_back(record);
        record_payloads.push_back(payload.data() + cursor);
        cursor += record.payload_size;
      }
      if (!records_valid || cursor != payload.size())
      {
        ++ReaderInternals::stats(reader).corrupt_block_count;
        ++ReaderInternals::stats(reader).skipped_block_count;
        ReaderInternals::stats(reader).skipped_record_count += records.size();
        if (options.strict)
        {
          if (error) *error = "invalid XLog record framing: " + path.string();
          return false;
        }
        continue;
      }

      ++ReaderInternals::stats(reader).block_count;
      for (std::size_t index = 0; index < records.size(); ++index)
      {
        const RecordHeader & record = records[index];
        if ((!have_sequence && record.sequence != 0) ||
            (have_sequence && record.sequence != previous_sequence + 1ULL))
        {
          ++ReaderInternals::stats(reader).sequence_gap_count;
          if (options.require_contiguous_sequence)
          {
            if (error) *error = "non-contiguous XLog record sequence";
            return false;
          }
        }
        previous_sequence = record.sequence;
        have_sequence = true;
        RecordView view;
        view.segment_index = expected_segment;
        view.file_id_hi = header.file_id_hi;
        view.file_id_lo = header.file_id_lo;
        view.header = record;
        view.payload = record_payloads[index];
        if (callback && !callback(view))
        {
          if (error) *error = "XLog record callback aborted";
          return false;
        }
        ++ReaderInternals::stats(reader).record_count;
      }
    }

    ReaderStats & stats = ReaderInternals::stats(reader);
    if (stats.segment_count == 0)
    {
      stats.footer_present = segment_footer_present;
      stats.footer_valid = segment_footer_valid;
    }
    else
    {
      stats.footer_present = stats.footer_present && segment_footer_present;
      stats.footer_valid = stats.footer_valid && segment_footer_valid;
    }
    ++stats.segment_count;
    if (options.require_footer && !segment_footer_valid)
    {
      if (error) *error = "XLog segment has no valid footer: " + path.string();
      return false;
    }
    return true;
  }
} // namespace

bool Reader::read(const std::string & path,
                  const RecordCallback & callback,
                  std::string * error,
                  const ReaderOptions & options)
{
  _metadata_json.clear();
  _schema_json.clear();
  _file_id_hi = 0;
  _file_id_lo = 0;
  _stats = {};
  if (path.empty())
  {
    if (error) *error = "XLog path is empty";
    return false;
  }
  if (options.max_block_payload_bytes < sizeof(RecordHeader))
  {
    if (error) *error = "XLog max_block_payload_bytes is too small";
    return false;
  }

  const Path base = base_path_for(Path(path));
  std::error_code exists_error;
  uint32_t first_segment = 0;
  if (!std::filesystem::exists(base, exists_error))
  {
    // Retention can remove the beginning of a run. Every segment carries a
    // complete preamble; start at the oldest retained part of this exact run.
    first_segment = 100000U;
    const auto parent = base.parent_path().empty() ? Path(".") : base.parent_path();
    for (const auto& entry : std::filesystem::directory_iterator(parent, exists_error))
    {
      if (has_part_suffix(entry.path()) && base_path_for(entry.path()) == base)
      {
        const auto stem = entry.path().stem().string();
        const auto index = static_cast<uint32_t>(std::stoul(stem.substr(stem.size() - 4))) - 1U;
        first_segment = std::min(first_segment, index);
      }
    }
    if (first_segment == 100000U)
    {
      if (error) *error = "XLog run has no retained segments: " + base.string();
      return false;
    }
  }
  uint64_t previous_sequence = 0;
  bool have_sequence = false;
  for (uint32_t segment = first_segment; segment < 100000U; ++segment)
  {
    const Path current(segment_path(base, segment));
    if (!std::filesystem::exists(current, exists_error))
    {
      if (segment == 0)
      {
        if (error) *error = "XLog path does not exist: " + current.string();
        return false;
      }
      break;
    }
    if (!read_segment(
        current, segment, *this, callback, options,
        previous_sequence, have_sequence, error))
      return false;
  }
  if (options.require_footer && !_stats.footer_valid)
  {
    if (error) *error = "XLog recording has no valid footer";
    return false;
  }
  return true;
}
} // namespace hydrox::xlog
