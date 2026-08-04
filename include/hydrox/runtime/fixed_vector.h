#pragma once

#include <array>
#include <cstddef>
#include <utility>

namespace hydrox::runtime
{
    /** Minimal vector-like container with compile-time storage and no heap use. */
    template <typename T, std::size_t Capacity>
    class FixedVector
    {
    public:
        using value_type = T;
        using iterator = T *;
        using const_iterator = const T *;

        constexpr std::size_t size() const noexcept { return size_; }
        constexpr std::size_t capacity() const noexcept { return Capacity; }
        constexpr bool empty() const noexcept { return size_ == 0; }
        constexpr bool full() const noexcept { return size_ == Capacity; }

        T *data() noexcept { return storage_.data(); }
        const T *data() const noexcept { return storage_.data(); }
        iterator begin() noexcept { return data(); }
        iterator end() noexcept { return data() + size_; }
        const_iterator begin() const noexcept { return data(); }
        const_iterator end() const noexcept { return data() + size_; }

        T &operator[](std::size_t index) noexcept { return storage_[index]; }
        const T &operator[](std::size_t index) const noexcept
        {
            return storage_[index];
        }

        void clear() noexcept { size_ = 0; }
        void reserve(std::size_t) noexcept {}

        bool push_back(const T &value)
        {
            if (full())
                return false;
            storage_[size_++] = value;
            return true;
        }

        bool push_back(T &&value)
        {
            if (full())
                return false;
            storage_[size_++] = std::move(value);
            return true;
        }

        void resize(std::size_t count, const T &value = T{})
        {
            if (count > Capacity)
                count = Capacity;
            while (size_ < count)
                storage_[size_++] = value;
            size_ = count;
        }

        template <typename InputIt>
        bool assign(InputIt first, InputIt last)
        {
            clear();
            bool complete = true;
            for (; first != last; ++first)
            {
                if (!push_back(*first))
                {
                    complete = false;
                    break;
                }
            }
            return complete;
        }

        iterator erase(const_iterator position)
        {
            return erase(position, position + 1);
        }

        iterator erase(const_iterator first, const_iterator last)
        {
            const std::size_t first_index =
                static_cast<std::size_t>(first - begin());
            const std::size_t last_index =
                static_cast<std::size_t>(last - begin());
            if (first_index >= size_ || last_index <= first_index)
                return begin() + (first_index < size_ ? first_index : size_);
            const std::size_t bounded_last =
                last_index < size_ ? last_index : size_;
            const std::size_t removed = bounded_last - first_index;
            for (std::size_t i = first_index; i + removed < size_; ++i)
                storage_[i] = std::move(storage_[i + removed]);
            size_ -= removed;
            return begin() + first_index;
        }

    private:
        std::array<T, Capacity> storage_{};
        std::size_t size_ = 0;
    };
} // namespace hydrox::runtime
