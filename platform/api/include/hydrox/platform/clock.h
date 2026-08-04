#pragma once

#include <cstdint>

namespace hydrox::platform
{
    using MonotonicTimeUs = uint64_t;

    /**
     * Monotonic clock contract shared by SITL and embedded targets.
     *
     * Time starts at an implementation-defined epoch and must never move
     * backwards. Runtime code must not use wall-clock time for scheduling.
     */
    class Clock
    {
    public:
        virtual ~Clock() = default;
        virtual MonotonicTimeUs now_us() const noexcept = 0;
    };
}
