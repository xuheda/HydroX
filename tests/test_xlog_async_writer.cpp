#include "xlog_async_writer.h"
#include "xlog_reader.h"
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <thread>

int main()
{
    using namespace hydrox::xlog;
    namespace fs = std::filesystem;
    const auto root = fs::temp_directory_path() / ("hydrox_async_" + std::to_string(unix_time_ns_now()));
    const auto path = (root / "flight.xlog").string();
    AsyncWriter writer;
    std::string error;
    if (!writer.open(path, "{}", &error)) return 1;
    HydroxStateRecord state;
    for (int i = 0; i < 100; ++i) writer.write(TopicId::HydroxState, i, state);
    writer.close();
    Reader reader;
    int records = 0;
    if (!reader.read(path, [&](const RecordView&) { ++records; return true; }, &error) || records != 100) return 2;
    if (!reader.stats().footer_valid || writer.dropped_records()) return 3;
    if (writer.open(path, "{}", &error)) return 4; // Cannot overwrite prior evidence.

    const auto overflow = (root / "overflow.xlog").string();
    if (!writer.open(overflow, "{}", &error)) return 5;
    for (int i = 0; i < 100000; ++i) writer.write(TopicId::HydroxState, i, state);
    HydroxSafetyEventRecord fault;
    fault.first_fault = 1;
    std::snprintf(fault.cause, sizeof(fault.cause), "SENSOR_TIMEOUT");
    if (!writer.write_critical(TopicId::HydroxSafetyEvent, 100001, fault)) return 6;
    writer.close();
    if (!writer.dropped_records() || !fs::exists(overflow + ".first_fault.json")) return 7;

    AsyncWriter::Limits disk;
    disk.reserve_bytes = std::numeric_limits<uint64_t>::max();
    if (!writer.open((root / "disk_full.xlog").string(), "{}", &error, disk)) return 8;
    writer.write(TopicId::HydroxState, 0, state);
    std::this_thread::sleep_for(std::chrono::milliseconds(1150));
    if (!writer.failed() || writer.last_error() != "diagnostic disk reserve reached") return 9;
    writer.close();
    // Rotation deletes only this writer's closed segments. Remaining parts
    // are still readable even after the original base file has been retired.
    AsyncWriter::Limits rotation;
    rotation.reserve_bytes = 0;
    rotation.writer.max_segment_bytes = 20000;
    rotation.writer.target_block_bytes = 2048;
    rotation.total_bytes = 45000;
    const auto rotated = (root / "rotate.xlog").string();
    if (!writer.open(rotated, "{}", &error, rotation)) return 10;
    for (int i = 0; i < 4000; ++i) writer.write(TopicId::HydroxState, i, state);
    std::this_thread::sleep_for(std::chrono::milliseconds(1150));
    writer.close();
    if (fs::exists(rotated)) return 11;
    if (!reader.read(rotated, [](const RecordView&) { return true; }, &error))
    { std::fprintf(stderr, "%s\n", error.c_str()); return 12; }
    std::printf("test_xlog_async_writer passed: automatic, drain, overflow, first fault, disk reserve, rotation\n");
    return 0;
}
