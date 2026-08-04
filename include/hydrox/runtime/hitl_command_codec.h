#pragma once
#include "hydrox/runtime/hitl_board.h"
#include <array>
#include <cstddef>
#include <cstdint>

namespace hydrox::runtime
{
    constexpr std::size_t kHitlCommandFrameSize = 54;

    struct HitlCommandFrame
    {
        HitlSetpointSample sample{};
        uint32_t sender_generation = 0;
        uint16_t sequence = 0;
        uint64_t sender_time_us = 0;
    };

    class HitlCommandDecoder
    {
    public:
        bool feed(const uint8_t *data, std::size_t size,
                  HitlCommandFrame &latest) noexcept;
        void reset() noexcept { used_ = 0; }

    private:
        bool decode(HitlCommandFrame &frame) const noexcept;
        void discard_first() noexcept;
        std::array<uint8_t, kHitlCommandFrameSize> buffer_{};
        std::size_t used_ = 0;
    };

    std::array<uint8_t, kHitlCommandFrameSize> encode_hitl_command(
        const HitlCommandFrame &frame) noexcept;
}