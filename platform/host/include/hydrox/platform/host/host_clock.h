#pragma once

#include "hydrox/platform/clock.h"

namespace hydrox::platform::host
{
    class HostClock final : public Clock
    {
    public:
        MonotonicTimeUs now_us() const noexcept override;
    };
}
