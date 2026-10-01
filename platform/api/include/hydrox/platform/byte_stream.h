#pragma once

#include <cstddef>
#include <cstdint>

namespace hydrox::platform
{
    enum class IoStatus : uint8_t
    {
        Ok = 0,
        WouldBlock,
        Closed,
        Error,
    };

    struct IoResult
    {
        std::size_t size = 0;
        IoStatus status = IoStatus::Error;

        constexpr bool ok() const noexcept
        {
            return status == IoStatus::Ok;
        }
    };

    /**
     * Non-blocking byte stream used by the embedded runtime.
     *
     * Implementations own all buffering. Calls never allocate memory and never
     * wait for an entire frame to arrive or leave the device.
     */
    class ByteStream
    {
    public:
        virtual ~ByteStream() = default;

        virtual bool open() noexcept = 0;
        virtual void close() noexcept = 0;
        virtual bool is_open() const noexcept = 0;

        virtual IoResult read(uint8_t *buffer, std::size_t capacity) noexcept = 0;
        virtual IoResult write(const uint8_t *data, std::size_t size) noexcept = 0;
    };
}
