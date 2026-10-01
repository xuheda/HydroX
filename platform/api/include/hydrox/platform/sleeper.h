#pragma once

#include "hydrox/platform/clock.h"

namespace hydrox::platform
{
    /**
     * Blocking wait primitive in the same monotonic time domain as Clock.
     */
    class Sleeper
    {
    public:
        virtual ~Sleeper() = default;

        virtual void sleep_for_us(MonotonicTimeUs duration_us) noexcept = 0;
        virtual void sleep_until_us(MonotonicTimeUs deadline_us) noexcept = 0;
    };
}
