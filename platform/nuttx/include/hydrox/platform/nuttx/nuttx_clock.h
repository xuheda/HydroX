#pragma once

#include "hydrox/platform/clock.h"

namespace hydrox::platform::nuttx
{
    class NuttxClock final : public Clock
    {
    public:
        MonotonicTimeUs now_us() const noexcept override;
    };
}
