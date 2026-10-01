#include "xlog_writer.h"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>

namespace
{
    int fail(const char *msg)
    {
        std::fprintf(stderr, "FAIL: %s\n", msg);
        return 1;
    }

    std::vector<uint8_t> read_file(const std::filesystem::path &path)
    {
        std::ifstream in(path, std::ios::binary);
        if (!in)
            return {};
        in.seekg(0, std::ios::end);
        const auto size = in.tellg();
        in.seekg(0, std::ios::beg);
        std::vector<uint8_t> bytes(static_cast<std::size_t>(size));
        in.read(reinterpret_cast<char *>(bytes.data()), size);
        return bytes;
    }

    template <typename T>
    bool read_struct(const std::vector<uint8_t> &bytes, std::size_t offset, T &out)
    {
        if (offset + sizeof(T) > bytes.size())
            return false;
        std::memcpy(&out, bytes.data() + offset, sizeof(T));
        return true;
    }
} // namespace

int main()
{
    using namespace hydrox::xlog;

    if (crc32("123456789", 9) != 0xCBF43926U)
        return fail("CRC32 reference vector");

    const std::filesystem::path dir =
        std::filesystem::temp_directory_path() /
        ("hydrox_test_xlog_v1_" + std::to_string(unix_time_ns_now()));
    const std::filesystem::path path = dir / "xlog_test_vehicle_20260715_120000.xlog";
    const std::string metadata = R"({"test":true,"vehicle":"test_vehicle"})";

    WriterOptions options;
    options.target_block_bytes = 256;
    options.max_segment_bytes =
        sizeof(FileHeader) + metadata.size() + default_schema_json().size() +
        sizeof(BlockHeader) + options.target_block_bytes + sizeof(FileFooter) + 32;

    Writer writer;
    std::string error;
    if (!writer.open(path.string(), metadata, &error, options))
        return fail(error.empty() ? "open" : error.c_str());

    for (uint64_t i = 0; i < 40; ++i)
    {
        HydroxControlErrorRecord record;
        record.depth_err = static_cast<double>(i) * 0.25;
        record.heading_err = -0.5;
        if (!writer.write(TopicId::HydroxControlError, 123456789ULL + i * 10000000ULL, record))
            return fail(writer.last_error().empty() ? "write record" : writer.last_error().c_str());
    }
    if (!writer.flush())
        return fail(writer.last_error().empty() ? "flush" : writer.last_error().c_str());
    writer.close();

    const std::vector<std::string> segments = writer.segment_paths();
    if (segments.size() < 2)
        return fail("segment rollover");

    uint64_t file_id_hi = 0;
    uint64_t file_id_lo = 0;
    uint64_t previous_next_sequence = 0;
    for (std::size_t segment_index = 0; segment_index < segments.size(); ++segment_index)
    {
        const std::vector<uint8_t> bytes = read_file(segments[segment_index]);
        if (bytes.size() < sizeof(FileHeader) + sizeof(BlockHeader) + sizeof(FileFooter))
            return fail("segment size");

        FileHeader header;
        if (!read_struct(bytes, 0, header))
            return fail("file header");
        if (std::memcmp(header.magic, "XLOG", 4) != 0 ||
            header.version_major != 1 || header.version_minor != 0 ||
            header.header_size != sizeof(FileHeader) ||
            header.block_header_size != sizeof(BlockHeader) ||
            header.record_header_size != sizeof(RecordHeader) ||
            header.segment_index != segment_index)
        {
            return fail("file header fields");
        }

        if (segment_index == 0)
        {
            file_id_hi = header.file_id_hi;
            file_id_lo = header.file_id_lo;
        }
        else if (header.file_id_hi != file_id_hi || header.file_id_lo != file_id_lo)
        {
            return fail("segment file id");
        }

        const std::size_t preamble_payload =
            static_cast<std::size_t>(header.metadata_len + header.schema_len);
        const std::size_t preamble_end = sizeof(FileHeader) + preamble_payload;
        if (preamble_end > bytes.size())
            return fail("preamble bounds");

        FileHeader header_for_crc = header;
        header_for_crc.header_crc32 = 0;
        uint32_t header_crc = crc32(&header_for_crc, sizeof(header_for_crc));
        header_crc = crc32(bytes.data() + sizeof(FileHeader), preamble_payload, header_crc);
        if (header_crc != header.header_crc32)
            return fail("preamble CRC");
        if (fnv1a64(bytes.data() + sizeof(FileHeader) + header.metadata_len,
                    static_cast<std::size_t>(header.schema_len)) != header.schema_hash)
        {
            return fail("schema hash");
        }

        BlockHeader block;
        if (!read_struct(bytes, preamble_end, block) || std::memcmp(block.magic, "XBLK", 4) != 0)
            return fail("block header");
        BlockHeader block_for_crc = block;
        block_for_crc.header_crc32 = 0;
        if (crc32(&block_for_crc, sizeof(block_for_crc)) != block.header_crc32)
            return fail("block header CRC");
        const std::size_t payload_offset = preamble_end + sizeof(BlockHeader);
        if (payload_offset + block.payload_size > bytes.size())
            return fail("block payload bounds");
        if (crc32(bytes.data() + payload_offset, block.payload_size) != block.payload_crc32)
            return fail("block payload CRC");

        RecordHeader record_header;
        if (!read_struct(bytes, payload_offset, record_header) ||
            record_header.topic_id != static_cast<uint16_t>(TopicId::HydroxControlError) ||
            record_header.payload_size != sizeof(HydroxControlErrorRecord))
        {
            return fail("record header");
        }

        std::vector<uint8_t> corrupted = bytes;
        corrupted[payload_offset + sizeof(RecordHeader)] ^= 0x01U;
        if (crc32(corrupted.data() + payload_offset, block.payload_size) == block.payload_crc32)
            return fail("payload corruption detection");

        FileFooter footer;
        if (!read_struct(bytes, bytes.size() - sizeof(FileFooter), footer) ||
            std::memcmp(footer.magic, "XEND", 4) != 0 ||
            footer.file_bytes != bytes.size())
        {
            return fail("file footer");
        }
        FileFooter footer_for_crc = footer;
        footer_for_crc.footer_crc32 = 0;
        if (crc32(&footer_for_crc, sizeof(footer_for_crc)) != footer.footer_crc32)
            return fail("footer CRC");
        if (footer.next_sequence <= previous_next_sequence)
            return fail("global segment sequence");
        previous_next_sequence = footer.next_sequence;
    }

    if (previous_next_sequence != 40)
        return fail("final sequence count");

    std::error_code cleanup_error;
    std::filesystem::remove_all(dir, cleanup_error);
    std::printf("test_xlog_writer: all checks passed (%zu segments)\n", segments.size());
    return 0;
}
