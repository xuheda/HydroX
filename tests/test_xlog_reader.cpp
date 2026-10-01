#include "xlog_reader.h"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace
{
  int fail(const char * message)
  {
    std::fprintf(stderr, "FAIL: %s\n", message);
    return 1;
  }

  std::vector<uint8_t> read_file(const std::filesystem::path & path)
  {
    std::ifstream input(path, std::ios::binary);
    input.seekg(0, std::ios::end);
    const auto length = input.tellg();
    input.seekg(0, std::ios::beg);
    std::vector<uint8_t> bytes(static_cast<std::size_t>(length));
    input.read(reinterpret_cast<char *>(bytes.data()), length);
    return bytes;
  }

  bool write_file(const std::filesystem::path & path,
                  const std::vector<uint8_t> & bytes)
  {
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    output.write(reinterpret_cast<const char *>(bytes.data()),
                 static_cast<std::streamsize>(bytes.size()));
    return output.good();
  }

  template<typename T>
  bool copy_struct(const std::vector<uint8_t> & bytes, std::size_t offset, T & value)
  {
    if (offset > bytes.size() || sizeof(T) > bytes.size() - offset)
      return false;
    std::memcpy(&value, bytes.data() + offset, sizeof(T));
    return true;
  }

  std::vector<std::size_t> block_offsets(const std::vector<uint8_t> & bytes)
  {
    using namespace hydrox::xlog;
    FileHeader file{};
    if (!copy_struct(bytes, 0, file))
      return {};
    std::size_t offset = sizeof(FileHeader) +
      static_cast<std::size_t>(file.metadata_len + file.schema_len);
    std::vector<std::size_t> result;
    while (offset + sizeof(BlockHeader) <= bytes.size())
    {
      BlockHeader block{};
      if (!copy_struct(bytes, offset, block) ||
          std::memcmp(block.magic, "XBLK", 4) != 0)
        break;
      result.push_back(offset);
      offset += sizeof(BlockHeader) + block.payload_size;
    }
    return result;
  }
} // namespace

int main()
{
  using namespace hydrox::xlog;
  const std::filesystem::path directory =
    std::filesystem::temp_directory_path() /
    ("hydrox_test_xlog_reader_" + std::to_string(unix_time_ns_now()));
  const std::filesystem::path source = directory / "reader.xlog";
  std::filesystem::create_directories(directory);

  WriterOptions writer_options;
  writer_options.target_block_bytes =
    sizeof(RecordHeader) + sizeof(HydroxControlErrorRecord);
  Writer writer;
  std::string error;
  if (!writer.open(source.string(), R"({"vehicle":"reader_test"})",
                   &error, writer_options))
    return fail(error.c_str());
  for (uint64_t index = 0; index < 5; ++index)
  {
    HydroxControlErrorRecord record;
    record.depth_err = static_cast<double>(index);
    if (!writer.write(TopicId::HydroxControlError, 1000 + index, record))
      return fail("write source record");
  }
  writer.close();

  Reader strict_reader;
  ReaderOptions strict_options;
  strict_options.strict = true;
  strict_options.require_footer = true;
  strict_options.require_contiguous_sequence = true;
  std::vector<double> values;
  if (!strict_reader.read(
      source.string(),
      [&](const RecordView & view)
      {
        if (view.header.topic_id !=
              static_cast<uint16_t>(TopicId::HydroxControlError) ||
            view.header.payload_size != sizeof(HydroxControlErrorRecord))
          return false;
        HydroxControlErrorRecord record{};
        std::memcpy(&record, view.payload, sizeof(record));
        values.push_back(record.depth_err);
        return true;
      },
      &error, strict_options))
    return fail(error.c_str());
  if (values.size() != 5 || values.front() != 0.0 || values.back() != 4.0)
    return fail("strict record decode");
  if (strict_reader.stats().block_count != 5 ||
      strict_reader.stats().record_count != 5 ||
      !strict_reader.stats().footer_present ||
      !strict_reader.stats().footer_valid ||
      strict_reader.metadata_json().find("reader_test") == std::string::npos)
    return fail("strict reader statistics/preamble");

  const std::vector<uint8_t> source_bytes = read_file(source);
  const auto offsets = block_offsets(source_bytes);
  if (offsets.size() != 5)
    return fail("source block layout");

  std::vector<uint8_t> corrupt_bytes = source_bytes;
  const std::size_t corrupt_payload = offsets[1] + sizeof(BlockHeader);
  corrupt_bytes[corrupt_payload + sizeof(RecordHeader)] ^= 0x80U;
  const std::filesystem::path corrupt_path = directory / "corrupt.xlog";
  if (!write_file(corrupt_path, corrupt_bytes))
    return fail("write corrupt fixture");

  Reader recovery_reader;
  if (!recovery_reader.read(corrupt_path.string(), {}, &error))
    return fail(error.c_str());
  if (recovery_reader.stats().record_count != 4 ||
      recovery_reader.stats().skipped_block_count != 1 ||
      recovery_reader.stats().skipped_record_count != 1 ||
      recovery_reader.stats().corrupt_block_count != 1 ||
      recovery_reader.stats().sequence_gap_count != 1)
    return fail("corrupt block recovery statistics");

  Reader reject_corrupt;
  if (reject_corrupt.read(corrupt_path.string(), {}, &error, strict_options))
    return fail("strict reader accepted corrupt block");

  BlockHeader last_block{};
  if (!copy_struct(source_bytes, offsets.back(), last_block))
    return fail("last block header");
  std::vector<uint8_t> truncated_bytes = source_bytes;
  truncated_bytes.resize(
    offsets.back() + sizeof(BlockHeader) + last_block.payload_size - 1U);
  const std::filesystem::path truncated_path = directory / "truncated.xlog";
  if (!write_file(truncated_path, truncated_bytes))
    return fail("write truncated fixture");

  Reader tail_reader;
  if (!tail_reader.read(truncated_path.string(), {}, &error))
    return fail(error.c_str());
  if (tail_reader.stats().record_count != 4 ||
      tail_reader.stats().truncated_tail_bytes == 0 ||
      tail_reader.stats().footer_present)
    return fail("truncated tail recovery");

  Reader require_footer_reader;
  ReaderOptions require_footer;
  require_footer.require_footer = true;
  if (require_footer_reader.read(
      truncated_path.string(), {}, &error, require_footer))
    return fail("footer requirement accepted truncated recording");

  const std::filesystem::path empty_path = directory / "empty.xlog";
  Writer empty_writer;
  if (!empty_writer.open(empty_path.string(), "{}", &error))
    return fail("open empty recording");
  empty_writer.close();
  Reader empty_reader;
  if (!empty_reader.read(empty_path.string(), {}, &error, strict_options) ||
      empty_reader.stats().record_count != 0 ||
      !empty_reader.stats().footer_valid)
    return fail("empty recording");

  std::error_code cleanup_error;
  std::filesystem::remove_all(directory, cleanup_error);
  std::printf("test_xlog_reader: all checks passed\n");
  return 0;
}
