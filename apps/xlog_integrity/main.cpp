#include "xlog_reader.h"

#include <cstdio>
#include <string>

int main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::fprintf(stderr, "usage: hydrox_xlog_integrity XLOG\n");
        return 2;
    }
    hydrox::xlog::Reader reader;
    hydrox::xlog::ReaderOptions options;
    options.strict = true;
    options.require_footer = true;
    options.require_contiguous_sequence = true;
    std::string error;
    const bool ok = reader.read(
        argv[1], [](const hydrox::xlog::RecordView&) { return true; },
        &error, options);
    if (!ok)
    {
        std::fprintf(stderr, "XLog integrity failed: %s\n", error.c_str());
        return 1;
    }
    const auto& stats = reader.stats();
    if (!stats.footer_present || !stats.footer_valid ||
        stats.corrupt_block_count != 0 || stats.sequence_gap_count != 0 ||
        stats.skipped_block_count != 0 || stats.skipped_record_count != 0 ||
        stats.truncated_tail_bytes != 0 || stats.record_count == 0)
    {
        std::fprintf(stderr, "XLog integrity counters are not clean\n");
        return 1;
    }
    std::printf(
        "{\"footer_present\":true,\"footer_valid\":true,"
        "\"segment_count\":%llu,\"block_count\":%llu,"
        "\"record_count\":%llu,\"skipped_block_count\":%llu,"
        "\"skipped_record_count\":%llu,\"corrupt_block_count\":%llu,"
        "\"sequence_gap_count\":%llu,\"truncated_tail_bytes\":%llu}\n",
        static_cast<unsigned long long>(stats.segment_count),
        static_cast<unsigned long long>(stats.block_count),
        static_cast<unsigned long long>(stats.record_count),
        static_cast<unsigned long long>(stats.skipped_block_count),
        static_cast<unsigned long long>(stats.skipped_record_count),
        static_cast<unsigned long long>(stats.corrupt_block_count),
        static_cast<unsigned long long>(stats.sequence_gap_count),
        static_cast<unsigned long long>(stats.truncated_tail_bytes));
    return 0;
}
