#pragma once

#include "hydrox/platform/byte_stream.h"

#include <cstdint>
#include <string>

namespace hydrox::platform::host
{
    /** Non-blocking 8-N-1 serial byte stream used by the desktop HITL router. */
    class HostSerialByteStream final : public ByteStream
    {
    public:
        HostSerialByteStream(std::string device, uint32_t baud_rate);
        ~HostSerialByteStream() override;

        HostSerialByteStream(const HostSerialByteStream &) = delete;
        HostSerialByteStream &operator=(const HostSerialByteStream &) = delete;

        bool open() noexcept override;
        void close() noexcept override;
        bool is_open() const noexcept override;
        IoResult read(uint8_t *buffer, std::size_t capacity) noexcept override;
        IoResult write(const uint8_t *data, std::size_t size) noexcept override;

        const std::string &device() const noexcept { return device_; }
        uint32_t baud_rate() const noexcept { return baud_rate_; }

    private:
        std::string device_;
        uint32_t baud_rate_ = 921600;
        std::intptr_t native_handle_ = -1;
    };
} // namespace hydrox::platform::host
