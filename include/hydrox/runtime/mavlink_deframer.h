#pragma once

#include "mavlink_hil.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace hydrox::runtime
{
    /** Heap-free MAVLink 2 packet boundary detector for transparent routers. */
    class MavlinkDeframer
    {
    public:
        template <typename FrameFn>
        void feed(const uint8_t *data, std::size_t size, FrameFn &&on_frame)
        {
            if (data == nullptr)
                return;
            for (std::size_t i = 0; i < size; ++i)
            {
                const uint8_t byte = data[i];
                if (length_ == 0 && byte != 0xFD)
                    continue;
                if (length_ >= buffer_.size())
                {
                    reset();
                    if (byte != 0xFD)
                        continue;
                }
                buffer_[length_++] = byte;
                if (length_ == 3)
                {
                    const bool signed_frame = (buffer_[2] & 0x01u) != 0;
                    expected_ = 10u + buffer_[1] + 2u +
                        (signed_frame ? MAVLINK_SIGNATURE_BLOCK_LEN : 0u);
                    if (expected_ > buffer_.size())
                        reset();
                }
                if (expected_ > 0 && length_ == expected_)
                {
                    on_frame(buffer_.data(), length_);
                    reset();
                }
            }
        }

        void reset() noexcept
        {
            length_ = 0;
            expected_ = 0;
        }

    private:
        std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer_{};
        std::size_t length_ = 0;
        std::size_t expected_ = 0;
    };
} // namespace hydrox::runtime
