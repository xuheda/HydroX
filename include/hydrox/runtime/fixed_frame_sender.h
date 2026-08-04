#pragma once

#include "hydrox/platform/byte_stream.h"
#include "hydrox/platform/clock.h"
#include "mavlink_hil.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace hydrox::runtime
{
    enum class FixedFrameSendStatus : uint8_t
    {
        COMPLETE = 0,
        TAIL_PENDING,
        FRAME_DROPPED,
        FATAL,
        TIMED_OUT,
        OVERSIZE,
    };

    /** Fixed-capacity, non-blocking frame sender for MCU byte streams. */
    class FixedFrameSender
    {
    public:
        explicit FixedFrameSender(
            platform::MonotonicTimeUs blocked_timeout_us = 500'000) noexcept
            : blocked_timeout_us_(blocked_timeout_us)
        {
        }

        FixedFrameSendStatus flush(platform::ByteStream &stream,
                                   platform::MonotonicTimeUs now_us) noexcept
        {
            while (pending_offset_ < pending_size_)
            {
                const std::size_t remaining = pending_size_ - pending_offset_;
                const platform::IoResult result = stream.write(
                    pending_.data() + pending_offset_, remaining);
                if (result.status == platform::IoStatus::Ok)
                {
                    if (result.size == 0 || result.size > remaining)
                        return FixedFrameSendStatus::FATAL;
                    pending_offset_ += result.size;
                    blocked_since_us_ = 0;
                    continue;
                }
                if (result.status == platform::IoStatus::WouldBlock)
                {
                    mark_blocked(now_us);
                    return timed_out(now_us)
                               ? FixedFrameSendStatus::TIMED_OUT
                               : FixedFrameSendStatus::TAIL_PENDING;
                }
                return FixedFrameSendStatus::FATAL;
            }

            pending_size_ = 0;
            pending_offset_ = 0;
            blocked_since_us_ = 0;
            return FixedFrameSendStatus::COMPLETE;
        }

        FixedFrameSendStatus write_frame(
            platform::ByteStream &stream,
            const uint8_t *data,
            std::size_t size,
            platform::MonotonicTimeUs now_us) noexcept
        {
            const FixedFrameSendStatus flushed = flush(stream, now_us);
            if (flushed == FixedFrameSendStatus::FATAL ||
                flushed == FixedFrameSendStatus::TIMED_OUT)
            {
                return flushed;
            }
            if (flushed == FixedFrameSendStatus::TAIL_PENDING)
            {
                ++dropped_frames_;
                return FixedFrameSendStatus::FRAME_DROPPED;
            }
            if (size > pending_.size())
                return FixedFrameSendStatus::OVERSIZE;
            if (size == 0)
                return FixedFrameSendStatus::COMPLETE;
            if (data == nullptr)
                return FixedFrameSendStatus::FATAL;

            std::size_t sent = 0;
            while (sent < size)
            {
                const std::size_t remaining = size - sent;
                const platform::IoResult result =
                    stream.write(data + sent, remaining);
                if (result.status == platform::IoStatus::Ok)
                {
                    if (result.size == 0 || result.size > remaining)
                        return FixedFrameSendStatus::FATAL;
                    sent += result.size;
                    blocked_since_us_ = 0;
                    continue;
                }
                if (result.status == platform::IoStatus::WouldBlock)
                {
                    mark_blocked(now_us);
                    if (timed_out(now_us))
                        return FixedFrameSendStatus::TIMED_OUT;
                    if (sent == 0)
                    {
                        ++dropped_frames_;
                        return FixedFrameSendStatus::FRAME_DROPPED;
                    }
                    pending_size_ = remaining;
                    pending_offset_ = 0;
                    std::memcpy(pending_.data(), data + sent, remaining);
                    return FixedFrameSendStatus::TAIL_PENDING;
                }
                return FixedFrameSendStatus::FATAL;
            }
            return FixedFrameSendStatus::COMPLETE;
        }

        void reset() noexcept
        {
            pending_size_ = 0;
            pending_offset_ = 0;
            blocked_since_us_ = 0;
            dropped_frames_ = 0;
        }

        std::size_t pending_bytes() const noexcept
        {
            return pending_size_ - pending_offset_;
        }
        uint64_t dropped_frames() const noexcept { return dropped_frames_; }

    private:
        void mark_blocked(platform::MonotonicTimeUs now_us) noexcept
        {
            if (blocked_since_us_ == 0)
                blocked_since_us_ = now_us == 0 ? 1 : now_us;
        }

        bool timed_out(platform::MonotonicTimeUs now_us) const noexcept
        {
            return blocked_timeout_us_ > 0 && blocked_since_us_ > 0 &&
                   now_us >= blocked_since_us_ &&
                   now_us - blocked_since_us_ >= blocked_timeout_us_;
        }

        std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> pending_{};
        std::size_t pending_size_ = 0;
        std::size_t pending_offset_ = 0;
        platform::MonotonicTimeUs blocked_since_us_ = 0;
        platform::MonotonicTimeUs blocked_timeout_us_ = 500'000;
        uint64_t dropped_frames_ = 0;
    };
} // namespace hydrox::runtime
