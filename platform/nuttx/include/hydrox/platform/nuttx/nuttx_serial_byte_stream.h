#pragma once
#include "hydrox/platform/byte_stream.h"
#include <cstdint>

namespace hydrox::platform::nuttx
{
    class NuttxSerialByteStream final : public ByteStream
    {
    public:
        NuttxSerialByteStream(const char *device, uint32_t baud,
                              bool hardware_flow_control) noexcept;
        ~NuttxSerialByteStream() override;
        bool open() noexcept override;
        void close() noexcept override;
        bool is_open() const noexcept override;
        IoResult read(uint8_t *buffer, std::size_t capacity) noexcept override;
        IoResult write(const uint8_t *data, std::size_t size) noexcept override;

    private:
        bool configure() noexcept;
        const char *device_;
        uint32_t baud_;
        bool hardware_flow_control_;
        int fd_ = -1;
    };
}