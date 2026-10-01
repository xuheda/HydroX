#pragma once

#include "xlog_writer.h"
#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <thread>
#include <cstdio>

namespace hydrox::xlog
{
// One producer (flight/transport loop), one consumer. No producer disk I/O,
// allocation, waiting for a lock, or waiting for disk completion.
class AsyncWriter
{
public:
    struct Limits
    {
        uint64_t total_bytes = 256ULL * 1024 * 1024;
        uint64_t reserve_bytes = 512ULL * 1024 * 1024;
        WriterOptions writer{64ULL * 1024 * 1024, 64U * 1024};
    };
    AsyncWriter() : queue_(new std::array<Entry, Capacity>) {}
    ~AsyncWriter() { close(); }
    AsyncWriter(const AsyncWriter&) = delete;
    AsyncWriter& operator=(const AsyncWriter&) = delete;

    bool open(const std::string& path, const std::string& metadata,
              std::string* error = nullptr)
    { return open(path, metadata, error, Limits{}); }
    bool open(const std::string& path, const std::string& metadata,
              std::string* error, const Limits& limits)
    {
        close();
        path_ = path; limits_ = limits; error_.clear();
        failed_ = false; dropped_ = 0; written_ = 0; bytes_ = 0;
        head_ = 0; tail_ = 0;
        std::error_code ec;
        if (std::filesystem::exists(path, ec))
        {
            error_ = "refusing to overwrite existing diagnostic log";
            failed_ = true;
        }
        else if (!writer_.open(path, metadata, &error_, limits.writer))
            failed_ = true;
        if (failed_)
        {
            if (error) *error = error_;
            std::fprintf(stderr, "[XLOG][ERROR] %s: %s\n", path_.c_str(), error_.c_str());
            return false;
        }
        // Header/schema must survive a failure before the first IMU sample.
        if (!writer_.flush())
        {
            error_ = writer_.last_error(); failed_ = true;
            if (error) *error = error_;
            return false;
        }
        running_ = true;
        worker_ = std::thread([this] { consume(); });
        return true;
    }
    bool is_open() const { return running_ && !failed_; }
    bool failed() const { return failed_; }
    const std::string& path() const { return path_; }
    std::string last_error() const
    { std::lock_guard<std::mutex> lock(error_mutex_); return error_; }
    uint64_t dropped_records() const { return dropped_; }
    template<class T> bool write(TopicId topic, uint64_t time, const T& value)
    {
        static_assert(sizeof(T) <= 576, "increase bounded XLog packet size");
        enqueue(topic, time, &value, sizeof(T), false);
        return !failed_; // Overflow is reported, never a flight-loop stall.
    }
    template<class T> bool write_critical(TopicId topic, uint64_t time, const T& value)
    { return enqueue(topic, time, &value, sizeof(T), true); }
    bool flush() { flush_requested_ = true; return !failed_; }
    void close()
    {
        running_ = false;
        if (worker_.joinable()) worker_.join();
        // Only used when opening failed before the worker was started.
        writer_.close();
    }

private:
    static constexpr size_t Capacity = 4096;
    struct Entry { TopicId topic{}; uint64_t time = 0; uint32_t size = 0;
                   std::array<unsigned char, 576> data{}; };
    bool enqueue(TopicId topic, uint64_t time, const void* data, uint32_t size, bool critical)
    {
        if (!is_open()) return false;
        const auto tail = tail_.load(std::memory_order_relaxed);
        if (tail - head_.load(std::memory_order_acquire) >= Capacity - (critical ? 0 : 64) || size > 576)
        { ++dropped_; return false; }
        Entry& e = (*queue_)[tail % Capacity];
        e.topic = topic; e.time = time; e.size = size;
        std::memcpy(e.data.data(), data, size);
        tail_.store(tail + 1, std::memory_order_release);
        return true;
    }
    void fail(const std::string& error)
    {
        { std::lock_guard<std::mutex> lock(error_mutex_); error_ = error; }
        failed_ = true;
        std::fprintf(stderr, "[XLOG][ERROR] %s: %s\n", path_.c_str(), error.c_str());
    }
    void status(const char* state)
    {
        std::ofstream out(path_ + ".status.json", std::ios::trunc);
        out << "{\"state\":\"" << state << "\",\"written_records\":" << written_
            << ",\"dropped_records\":" << dropped_.load()
            << ",\"error\":\"" << json_escape(last_error()) << "\"}\n";
        if (!out) std::fprintf(stderr, "[XLOG][ERROR] cannot persist log status: %s\n", path_.c_str());
    }
    void consume()
    {
        try
        {
            auto last_flush = std::chrono::steady_clock::now();
            auto last_status = last_flush;
            size_t first_retained = 0;
            bool first_fault_saved = false;
            uint64_t reported_drops = 0;
            status("recording");
            for (;;)
            {
                Entry e;
                bool have = false;
                const auto head = head_.load(std::memory_order_relaxed);
                if (head != tail_.load(std::memory_order_acquire))
                {
                    e = (*queue_)[head % Capacity];
                    head_.store(head + 1, std::memory_order_release); have = true;
                }
                if (!have && !running_) break;
                if (have && !failed_)
                {
                    if (!writer_.write(e.topic, e.time, e.data.data(), e.size)) fail(writer_.last_error());
                    else ++written_;
                    if (e.topic == TopicId::HydroxSafetyEvent && e.size == sizeof(HydroxSafetyEventRecord))
                    {
                        HydroxSafetyEventRecord fault;
                        std::memcpy(&fault, e.data.data(), sizeof(fault));
                        if (fault.first_fault && !first_fault_saved)
                        {
                            std::ofstream first(path_ + ".first_fault.json", std::ios::trunc);
                            first << "{\"wall_unix_ns\":" << fault.wall_unix_ns
                                  << ",\"sensor_us\":" << fault.sensor_us
                                  << ",\"monotonic_us\":" << fault.monotonic_us
                                  << ",\"cause\":\"" << json_escape(fault.cause)
                                  << "\",\"reason\":\"" << json_escape(fault.reason)
                                  << "\",\"mode\":\"" << json_escape(fault.mode)
                                  << "\",\"observed_s\":" << fault.observed_s
                                  << ",\"threshold_s\":" << fault.threshold_s << "}\n";
                            first.flush();
                            if (!first) fail("cannot preserve first fault summary");
                            else first_fault_saved = true;
                        }
                    }
                }
                if (have && failed_) ++dropped_;
                const auto now = std::chrono::steady_clock::now();
                if (now - last_flush >= std::chrono::seconds(1) || flush_requested_.exchange(false))
                {
                    if (!failed_ && !writer_.flush()) fail(writer_.last_error());
                    last_flush = now;
                }
                if (now - last_status >= std::chrono::seconds(1))
                {
                    // Only prune closed segments created by this writer, never arbitrary files.
                    const auto& segments = writer_.segment_paths();
                    uint64_t retained = 0;
                    std::error_code ec;
                    for (size_t i = first_retained; i < segments.size(); ++i)
                    {
                        const auto size = std::filesystem::file_size(segments[i], ec);
                        if (!ec) retained += size;
                    }
                    while (retained > limits_.total_bytes && first_retained + 1 < segments.size())
                    {
                        const auto& oldest = segments[first_retained];
                        const auto size = std::filesystem::file_size(oldest, ec);
                        if (ec || !std::filesystem::remove(oldest, ec) || ec)
                        { fail("cannot rotate closed diagnostic segment"); break; }
                        retained -= size; ++first_retained;
                    }
                    const auto space = std::filesystem::space(std::filesystem::absolute(path_).parent_path(), ec);
                    if (!failed_ && (ec || space.available < limits_.reserve_bytes))
                        fail(ec ? "cannot inspect diagnostic disk space" : "diagnostic disk reserve reached");
                    const auto drops = dropped_.load();
                    if (drops != reported_drops)
                    {
                        std::fprintf(stderr, "[XLOG][WARNING] %s dropped_records=%llu\n", path_.c_str(),
                                     static_cast<unsigned long long>(drops));
                        reported_drops = drops;
                    }
                    status(failed_ ? "failed" : "recording"); last_status = now;
                }
                if (!have) std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            writer_.close();
            if (writer_.failed() && !failed_) fail(writer_.last_error());
        }
        catch (const std::exception& e) { fail(e.what()); }
        status(failed_ ? "failed" : (dropped_ ? "complete_with_drops" : "complete"));
    }
    Writer writer_;
    Limits limits_{};
    std::unique_ptr<std::array<Entry, Capacity>> queue_;
    mutable std::mutex error_mutex_;
    std::atomic<uint64_t> head_{0}, tail_{0};
    std::atomic<bool> running_{false}, failed_{false}, flush_requested_{false};
    std::atomic<uint64_t> dropped_{0};
    uint64_t written_ = 0, bytes_ = 0;
    std::thread worker_;
    std::string path_, error_;
};
} // namespace hydrox::xlog
