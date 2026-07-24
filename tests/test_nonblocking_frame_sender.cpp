#include "nonblocking_frame_sender.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <initializer_list>
#include <vector>

namespace
{
    using hydrox::detail::FrameSendStatus;
    using hydrox::detail::NonblockingFrameSender;
    using hydrox::detail::SendAttempt;

    void require(bool condition, const char *message)
    {
        if (!condition)
        {
            std::fprintf(stderr, "FAIL: %s\n", message);
            std::exit(1);
        }
    }

    enum class StepKind
    {
        Send,
        WouldBlock,
        Fatal
    };

    struct Step
    {
        StepKind kind;
        size_t bytes = 0;
    };

    class ScriptedSender
    {
    public:
        ScriptedSender(std::vector<uint8_t> &wire,
                       std::initializer_list<Step> steps)
            : wire_(wire), steps_(steps)
        {
        }

        SendAttempt operator()(const uint8_t *data, size_t length)
        {
            require(!steps_.empty(), "send script exhausted");
            const Step step = steps_.front();
            steps_.pop_front();

            if (step.kind == StepKind::WouldBlock)
                return SendAttempt::would_block();
            if (step.kind == StepKind::Fatal)
                return SendAttempt::fatal();

            const size_t count = std::min(step.bytes, length);
            require(count > 0, "scripted send must make progress");
            wire_.insert(wire_.end(), data, data + count);
            return SendAttempt::progress(count);
        }

    private:
        std::vector<uint8_t> &wire_;
        std::deque<Step> steps_;
    };

    void test_partial_tail_is_completed_before_next_frame()
    {
        NonblockingFrameSender sender;
        std::vector<uint8_t> wire;
        const std::vector<uint8_t> first{0xFD, 1, 2, 3, 4, 5};
        const std::vector<uint8_t> second{0xFD, 9, 8};
        const auto now = NonblockingFrameSender::Clock::now();

        ScriptedSender partial(wire, {
                                           {StepKind::Send, 3},
                                           {StepKind::WouldBlock, 0},
                                       });
        require(sender.write_frame(first.data(), first.size(), now, partial) ==
                    FrameSendStatus::TailPending,
                "partial send should retain the frame tail");
        require(sender.pending_bytes() == 3, "exact unsent tail length should be retained");

        ScriptedSender still_blocked(wire, {{StepKind::WouldBlock, 0}});
        require(sender.write_frame(second.data(), second.size(), now, still_blocked) ==
                    FrameSendStatus::FrameDropped,
                "a new frame must not be inserted before the old tail");
        require(sender.pending_bytes() == 3, "dropping a new frame must preserve the old tail");

        ScriptedSender finish_tail(wire, {{StepKind::Send, 3}});
        require(sender.flush(now, finish_tail) == FrameSendStatus::Complete,
                "old frame tail should flush completely");

        ScriptedSender send_second(wire, {{StepKind::Send, second.size()}});
        require(sender.write_frame(second.data(), second.size(), now, send_second) ==
                    FrameSendStatus::Complete,
                "next frame should send after the old tail is complete");

        std::vector<uint8_t> expected = first;
        expected.insert(expected.end(), second.begin(), second.end());
        require(wire == expected, "wire bytes must contain two intact ordered frames");
        require(sender.dropped_frames() == 1, "safe whole-frame drop should be counted");
    }

    void test_zero_byte_would_block_drops_only_untouched_frame()
    {
        NonblockingFrameSender sender;
        std::vector<uint8_t> wire;
        const std::vector<uint8_t> stale{1, 2, 3};
        const std::vector<uint8_t> latest{4, 5, 6};
        const auto now = NonblockingFrameSender::Clock::now();

        ScriptedSender blocked(wire, {{StepKind::WouldBlock, 0}});
        require(sender.write_frame(stale.data(), stale.size(), now, blocked) ==
                    FrameSendStatus::FrameDropped,
                "an untouched frame may be safely dropped under backpressure");
        require(sender.pending_bytes() == 0, "untouched dropped frame must not become a FIFO entry");

        ScriptedSender available(wire, {{StepKind::Send, latest.size()}});
        require(sender.write_frame(latest.data(), latest.size(), now, available) ==
                    FrameSendStatus::Complete,
                "latest frame should send when capacity returns");
        require(wire == latest, "dropped stale frame must contribute no stream bytes");
    }

    void test_persistent_backpressure_times_out()
    {
        NonblockingFrameSender sender(std::chrono::milliseconds(500));
        std::vector<uint8_t> wire;
        const std::vector<uint8_t> frame{1, 2, 3};
        const auto start = NonblockingFrameSender::Clock::now();

        ScriptedSender first_block(wire, {{StepKind::WouldBlock, 0}});
        require(sender.write_frame(frame.data(), frame.size(), start, first_block) ==
                    FrameSendStatus::FrameDropped,
                "first backpressure event should not immediately disconnect");

        ScriptedSender timed_block(wire, {{StepKind::WouldBlock, 0}});
        require(sender.write_frame(frame.data(), frame.size(),
                                   start + std::chrono::milliseconds(500), timed_block) ==
                    FrameSendStatus::TimedOut,
                "persistent backpressure should force reconnect after timeout");
    }

    void test_reset_discards_previous_connection_tail()
    {
        NonblockingFrameSender sender;
        std::vector<uint8_t> wire;
        const std::vector<uint8_t> old_frame{1, 2, 3, 4};
        const std::vector<uint8_t> new_frame{7, 8};
        const auto now = NonblockingFrameSender::Clock::now();

        ScriptedSender partial(wire, {
                                           {StepKind::Send, 2},
                                           {StepKind::WouldBlock, 0},
                                       });
        require(sender.write_frame(old_frame.data(), old_frame.size(), now, partial) ==
                    FrameSendStatus::TailPending,
                "old connection should have a pending tail before reset");

        sender.reset();
        require(sender.pending_bytes() == 0, "reconnect reset must discard old stream tail");

        wire.clear(); // A new TCP connection has a fresh byte stream.
        ScriptedSender send_new(wire, {{StepKind::Send, new_frame.size()}});
        require(sender.write_frame(new_frame.data(), new_frame.size(), now, send_new) ==
                    FrameSendStatus::Complete,
                "new connection should start with the new frame");
        require(wire == new_frame, "old connection tail must never replay after reconnect");
    }

    void test_fatal_send_is_propagated()
    {
        NonblockingFrameSender sender;
        std::vector<uint8_t> wire;
        const std::vector<uint8_t> frame{1};
        ScriptedSender fatal(wire, {{StepKind::Fatal, 0}});

        require(sender.write_frame(frame.data(), frame.size(),
                                   NonblockingFrameSender::Clock::now(), fatal) ==
                    FrameSendStatus::Fatal,
                "fatal socket error should be propagated to TcpTransport");
    }
}

int main()
{
    test_partial_tail_is_completed_before_next_frame();
    test_zero_byte_would_block_drops_only_untouched_frame();
    test_persistent_backpressure_times_out();
    test_reset_discards_previous_connection_tail();
    test_fatal_send_is_propagated();
    std::puts("PASS: non-blocking frame sender");
    return 0;
}
