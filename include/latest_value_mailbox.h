#pragma once

#include <cstdint>
#include <mutex>
#include <utility>

namespace hydrox
{
    // Thread-safe single-slot mailbox. Writers replace stale data instead of
    // building a queue. Real-time callers use try_publish/try_take_if_new and
    // therefore never wait for the other thread.
    template <typename T>
    class LatestValueMailbox
    {
    public:
        LatestValueMailbox() = default;
        LatestValueMailbox(const LatestValueMailbox &) = delete;
        LatestValueMailbox &operator=(const LatestValueMailbox &) = delete;

        void publish(const T &value)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            value_ = value;
            ++sequence_;
        }

        bool try_publish(const T &value)
        {
            std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
            if (!lock.owns_lock())
                return false;
            value_ = value;
            ++sequence_;
            return true;
        }

        bool try_publish(T &&value)
        {
            std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
            if (!lock.owns_lock())
                return false;
            value_ = std::move(value);
            ++sequence_;
            return true;
        }

        bool try_take_if_new(uint64_t &last_sequence, T &out) const
        {
            std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
            if (!lock.owns_lock() || sequence_ == 0 || sequence_ == last_sequence)
                return false;
            out = value_;
            last_sequence = sequence_;
            return true;
        }

    private:
        mutable std::mutex mutex_;
        T value_{};
        uint64_t sequence_{0};
    };
} // namespace hydrox
