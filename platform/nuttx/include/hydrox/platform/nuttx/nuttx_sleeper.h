#pragma once

#include "hydrox/platform/sleeper.h"

namespace hydrox::platform::nuttx
{
    class NuttxSleeper final : public Sleeper
    {
    public:
        void sleep_for_us(MonotonicTimeUs duration_us) noexcept override;
        void sleep_until_us(MonotonicTimeUs deadline_us) noexcept override;
    };
}
