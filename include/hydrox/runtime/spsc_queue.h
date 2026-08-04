#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <type_traits>

namespace hydrox::runtime
{
    /**
     * Bounded single-producer/single-consumer event queue.
     *
     * Capacity is exact: SpscQueue<T, 8> stores eight values. Publishing never
     * allocates and returns false instead of overwriting unread data.
     */
    template <typename T, std::size_t Capacity>
    class SpscQueue
    {
        static_assert(Capacity > 0, "HydroBus queue capacity must be positive");
        static_assert(std::is_copy_assignable<T>::value,
                      "HydroBus queue values must be copy assignable");

    public:
        bool try_push(const T &value) noexcept
        {
            const std::size_t head = head_.load(std::memory_order_relaxed);
            const std::size_t tail = tail_.load(std::memory_order_acquire);
            if (head - tail >= Capacity)
                return false;

            values_[head % Capacity] = value;
            head_.store(head + 1, std::memory_order_release);
            return true;
        }

        bool try_pop(T &value) noexcept
        {
            const std::size_t tail = tail_.load(std::memory_order_relaxed);
            const std::size_t head = head_.load(std::memory_order_acquire);
            if (tail == head)
                return false;

            value = values_[tail % Capacity];
            tail_.store(tail + 1, std::memory_order_release);
            return true;
        }

        std::size_t size_approx() const noexcept
        {
            const std::size_t head = head_.load(std::memory_order_acquire);
            const std::size_t tail = tail_.load(std::memory_order_acquire);
            return head - tail;
        }

        constexpr std::size_t capacity() const noexcept
        {
            return Capacity;
        }

    private:
        std::array<T, Capacity> values_{};
        std::atomic<std::size_t> head_{0};
        std::atomic<std::size_t> tail_{0};
    };
}
