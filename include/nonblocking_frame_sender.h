#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace hydrox::detail
{

    enum class SendAttemptKind
    {
        Progress,
        WouldBlock,
        Fatal
    };

    struct SendAttempt
    {
        SendAttemptKind kind = SendAttemptKind::Fatal;
        size_t bytes = 0;

        static SendAttempt progress(size_t count)
        {
            return {SendAttemptKind::Progress, count};
        }

        static SendAttempt would_block()
        {
            return {SendAttemptKind::WouldBlock, 0};
        }

        static SendAttempt fatal()
        {
            return {SendAttemptKind::Fatal, 0};
        }
    };

    enum class FrameSendStatus
    {
        Complete,
        TailPending,
        FrameDropped,
        Fatal,
        TimedOut
    };

    /**
     * State machine for writing complete frames to a non-blocking byte stream.
     *
     * Once any prefix of a frame reaches the stream, the unsent tail is retained
     * and must be flushed before another frame can start. Frames arriving while
     * that tail is still blocked are dropped before any of their bytes are sent,
     * so stream framing remains intact without building a stale command FIFO.
     */
    class NonblockingFrameSender
    {
    public:
        using Clock = std::chrono::steady_clock;

        explicit NonblockingFrameSender(
            std::chrono::milliseconds blocked_timeout = std::chrono::milliseconds(500))
            : blocked_timeout_(blocked_timeout)
        {
        }

        template <typename SendFn>
        FrameSendStatus flush(Clock::time_point now, SendFn &&send_fn)
        {
            while (pending_offset_ < pending_tail_.size())
            {
                const size_t remaining = pending_tail_.size() - pending_offset_;
                const SendAttempt attempt =
                    send_fn(pending_tail_.data() + pending_offset_, remaining);

                if (attempt.kind == SendAttemptKind::Progress)
                {
                    if (attempt.bytes == 0 || attempt.bytes > remaining)
                        return FrameSendStatus::Fatal;

                    pending_offset_ += attempt.bytes;
                    if (pending_offset_ == pending_tail_.size())
                    {
                        clear_pending();
                        blocked_since_.reset();
                        return FrameSendStatus::Complete;
                    }
                    continue;
                }

                if (attempt.kind == SendAttemptKind::WouldBlock)
                {
                    mark_blocked(now);
                    return has_timed_out(now)
                               ? FrameSendStatus::TimedOut
                               : FrameSendStatus::TailPending;
                }

                return FrameSendStatus::Fatal;
            }

            return FrameSendStatus::Complete;
        }

        template <typename SendFn>
        FrameSendStatus write_frame(const uint8_t *data,
                                    size_t length,
                                    Clock::time_point now,
                                    SendFn &&send_fn)
        {
            const FrameSendStatus flush_status = flush(now, send_fn);
            if (flush_status == FrameSendStatus::Fatal ||
                flush_status == FrameSendStatus::TimedOut)
            {
                return flush_status;
            }
            if (flush_status == FrameSendStatus::TailPending)
            {
                ++dropped_frames_;
                return FrameSendStatus::FrameDropped;
            }

            if (length == 0)
            {
                blocked_since_.reset();
                return FrameSendStatus::Complete;
            }
            if (data == nullptr)
                return FrameSendStatus::Fatal;

            size_t sent = 0;
            while (sent < length)
            {
                const size_t remaining = length - sent;
                const SendAttempt attempt = send_fn(data + sent, remaining);

                if (attempt.kind == SendAttemptKind::Progress)
                {
                    if (attempt.bytes == 0 || attempt.bytes > remaining)
                        return FrameSendStatus::Fatal;
                    sent += attempt.bytes;
                    continue;
                }

                if (attempt.kind == SendAttemptKind::WouldBlock)
                {
                    mark_blocked(now);
                    if (has_timed_out(now))
                        return FrameSendStatus::TimedOut;

                    if (sent == 0)
                    {
                        // No byte from this frame entered the stream, so dropping
                        // the complete frame cannot damage stream framing.
                        ++dropped_frames_;
                        return FrameSendStatus::FrameDropped;
                    }

                    pending_tail_.assign(data + sent, data + length);
                    pending_offset_ = 0;
                    return FrameSendStatus::TailPending;
                }

                return FrameSendStatus::Fatal;
            }

            blocked_since_.reset();
            return FrameSendStatus::Complete;
        }

        void reset()
        {
            clear_pending();
            blocked_since_.reset();
            dropped_frames_ = 0;
        }

        size_t pending_bytes() const
        {
            return pending_tail_.size() - pending_offset_;
        }

        uint64_t dropped_frames() const { return dropped_frames_; }

    private:
        void clear_pending()
        {
            pending_tail_.clear();
            pending_offset_ = 0;
        }

        void mark_blocked(Clock::time_point now)
        {
            if (!blocked_since_)
                blocked_since_ = now;
        }

        bool has_timed_out(Clock::time_point now) const
        {
            return blocked_since_ && now - *blocked_since_ >= blocked_timeout_;
        }

        std::chrono::milliseconds blocked_timeout_;
        std::vector<uint8_t> pending_tail_;
        size_t pending_offset_ = 0;
        std::optional<Clock::time_point> blocked_since_;
        uint64_t dropped_frames_ = 0;
    };

} // namespace hydrox::detail
