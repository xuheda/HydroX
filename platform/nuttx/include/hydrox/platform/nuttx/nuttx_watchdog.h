#pragma once
#include "hydrox/platform/watchdog.h"

namespace hydrox::platform::nuttx
{
    class NuttxWatchdog final : public Watchdog
    {
    public:
        explicit NuttxWatchdog(const char *device = "/dev/watchdog0") noexcept;
        ~NuttxWatchdog() override;
        bool start(uint32_t timeout_ms) noexcept override;
        void kick() noexcept override;
        bool is_running() const noexcept override;

    private:
        const char *device_;
        int fd_ = -1;
        bool running_ = false;
    };
}