#pragma once

namespace hydrox::platform
{
    class CriticalSection
    {
    public:
        virtual ~CriticalSection() = default;
        virtual void lock() noexcept = 0;
        virtual void unlock() noexcept = 0;
    };

    class CriticalSectionGuard
    {
    public:
        explicit CriticalSectionGuard(CriticalSection &section) noexcept
            : section_(section)
        {
            section_.lock();
        }

        ~CriticalSectionGuard()
        {
            section_.unlock();
        }

        CriticalSectionGuard(const CriticalSectionGuard &) = delete;
        CriticalSectionGuard &operator=(const CriticalSectionGuard &) = delete;

    private:
        CriticalSection &section_;
    };
}
