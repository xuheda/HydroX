#pragma once

#include <cstdint>

namespace hydrox::platform
{
    class Watchdog
    {
    public:
        virtual ~Watchdog() = default;

        virtual bool start(uint32_t timeout_ms) noexcept = 0;
        virtual void kick() noexcept = 0;
        virtual bool is_running() const noexcept = 0;
    };
}
