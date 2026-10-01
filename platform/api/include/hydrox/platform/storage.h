#pragma once

#include <cstddef>
#include <cstdint>

namespace hydrox::platform
{
    enum class StorageStatus : uint8_t
    {
        Ok = 0,
        NotFound,
        NoSpace,
        InvalidArgument,
        IoError,
    };

    struct StorageReadResult
    {
        std::size_t size = 0;
        StorageStatus status = StorageStatus::IoError;
    };

    /**
     * Small fixed-key storage contract for parameters and calibration data.
     *
     * File paths deliberately do not appear in this API. The host
     * implementation may use files while the board implementation may use
     * flash, FRAM, or another persistent device.
     */
    class KeyValueStorage
    {
    public:
        virtual ~KeyValueStorage() = default;

        virtual StorageReadResult read(
            uint32_t key,
            void *buffer,
            std::size_t capacity) noexcept = 0;

        virtual StorageStatus write(
            uint32_t key,
            const void *data,
            std::size_t size) noexcept = 0;

        virtual StorageStatus erase(uint32_t key) noexcept = 0;
        virtual StorageStatus sync() noexcept = 0;
    };
}
