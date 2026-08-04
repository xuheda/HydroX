#pragma once

#include "hydrox/platform/critical_section.h"

#include <mutex>

namespace hydrox::platform::host
{
    class HostCriticalSection final : public CriticalSection
    {
    public:
        void lock() noexcept override
        {
            mutex_.lock();
        }

        void unlock() noexcept override
        {
            mutex_.unlock();
        }

    private:
        std::mutex mutex_;
    };
}
