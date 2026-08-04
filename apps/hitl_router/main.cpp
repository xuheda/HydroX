#include "hydrox/platform/host/host_clock.h"
#include "hydrox/platform/host/host_serial_byte_stream.h"
#include "hydrox/platform/host/host_sleeper.h"
#include "hydrox/runtime/fixed_frame_sender.h"
#include "hydrox/runtime/mavlink_deframer.h"
#include "tcp_transport.h"

#include <array>
#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace
{
    std::atomic<bool> running{true};

    struct Config
    {
        std::string ue_host = "127.0.0.1";
        uint16_t ue_port = 14600;
        std::string serial_device;
        uint32_t baud_rate = 921600;
    };

    void signal_handler(int)
    {
        running = false;
    }

    bool parse_unsigned(const char *text, uint64_t maximum, uint64_t &value)
    {
        if (text == nullptr || *text == '\0')
            return false;
        char *end = nullptr;
        const unsigned long long parsed = std::strtoull(text, &end, 10);
        if (end == text || *end != '\0' || parsed > maximum)
            return false;
        value = static_cast<uint64_t>(parsed);
        return true;
    }

    bool parse_config(int argc, char *argv[], Config &config)
    {
        for (int i = 1; i < argc; ++i)
        {
            const char *key = argv[i];
            if (std::strcmp(key, "--help") == 0)
                return false;
            if (i + 1 >= argc)
                return false;
            const char *value = argv[++i];
            if (std::strcmp(key, "--ue-host") == 0)
                config.ue_host = value;
            else if (std::strcmp(key, "--serial") == 0)
                config.serial_device = value;
            else if (std::strcmp(key, "--ue-port") == 0)
            {
                uint64_t parsed = 0;
                if (!parse_unsigned(value, 65535, parsed) || parsed == 0)
                    return false;
                config.ue_port = static_cast<uint16_t>(parsed);
            }
            else if (std::strcmp(key, "--baud") == 0)
            {
                uint64_t parsed = 0;
                if (!parse_unsigned(value, 4'000'000, parsed) || parsed == 0)
                    return false;
                config.baud_rate = static_cast<uint32_t>(parsed);
            }
            else
            {
                return false;
            }
        }
        return !config.serial_device.empty();
    }

    void usage(const char *program)
    {
        std::fprintf(
            stderr,
            "Usage: %s --serial <COM7|/dev/ttyACM0> "
            "[--baud 921600] [--ue-host 127.0.0.1] [--ue-port 14600]\n",
            program != nullptr ? program : "hydrox_hitl_router");
    }
}

int main(int argc, char *argv[])
{
    Config config;
    if (!parse_config(argc, argv, config))
    {
        usage(argc > 0 ? argv[0] : nullptr);
        return 2;
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    hydrox::platform::host::HostClock clock;
    hydrox::platform::host::HostSleeper sleeper;
    hydrox::platform::host::HostSerialByteStream serial(
        config.serial_device, config.baud_rate);
    hydrox::TcpTransport ue(config.ue_host, config.ue_port, true);
    std::array<uint8_t, 1024> buffer{};

    std::printf(
        "[HITL Router] UE=%s:%u serial=%s@%u\n",
        config.ue_host.c_str(),
        static_cast<unsigned>(config.ue_port),
        config.serial_device.c_str(),
        static_cast<unsigned>(config.baud_rate));

    while (running)
    {
        if (!serial.open())
        {
            std::fprintf(stderr, "[HITL Router] serial open failed; retrying\n");
            sleeper.sleep_for_us(1'000'000);
            continue;
        }
        if (!ue.connect())
        {
            std::fprintf(stderr, "[HITL Router] UE connect failed; retrying\n");
            serial.close();
            sleeper.sleep_for_us(1'000'000);
            continue;
        }

        hydrox::runtime::MavlinkDeframer from_ue;
        hydrox::runtime::MavlinkDeframer from_board;
        hydrox::runtime::FixedFrameSender serial_sender;
        bool link_ok = true;
        std::printf("[HITL Router] bridge active\n");

        while (running && link_ok && ue.is_connected() && serial.is_open())
        {
            bool progressed = false;
            const auto now_us = clock.now_us();
            const auto flush_status = serial_sender.flush(serial, now_us);
            if (flush_status == hydrox::runtime::FixedFrameSendStatus::FATAL ||
                flush_status == hydrox::runtime::FixedFrameSendStatus::TIMED_OUT)
            {
                link_ok = false;
                break;
            }

            const int readable = ue.wait_readable(1);
            if (readable < 0)
            {
                link_ok = false;
                break;
            }
            if (readable > 0)
            {
                const int bytes = ue.read(buffer.data(), buffer.size());
                if (bytes < 0)
                {
                    link_ok = false;
                    break;
                }
                if (bytes > 0)
                {
                    progressed = true;
                    from_ue.feed(
                        buffer.data(), static_cast<std::size_t>(bytes),
                        [&](const uint8_t *frame, std::size_t size)
                        {
                            const auto status = serial_sender.write_frame(
                                serial, frame, size, clock.now_us());
                            if (status ==
                                    hydrox::runtime::FixedFrameSendStatus::FATAL ||
                                status ==
                                    hydrox::runtime::FixedFrameSendStatus::TIMED_OUT ||
                                status ==
                                    hydrox::runtime::FixedFrameSendStatus::OVERSIZE)
                            {
                                link_ok = false;
                            }
                        });
                }
            }

            const hydrox::platform::IoResult serial_read = serial.read(
                buffer.data(), buffer.size());
            if (serial_read.status == hydrox::platform::IoStatus::Ok)
            {
                progressed = true;
                from_board.feed(
                    buffer.data(), serial_read.size,
                    [&](const uint8_t *frame, std::size_t size)
                    {
                        if (!ue.write(frame, size))
                            link_ok = false;
                    });
            }
            else if (serial_read.status !=
                     hydrox::platform::IoStatus::WouldBlock)
            {
                link_ok = false;
            }

            if (!progressed)
                sleeper.sleep_for_us(1'000);
        }

        ue.disconnect();
        serial.close();
        std::fprintf(stderr, "[HITL Router] link lost; reconnecting\n");
        if (running)
            sleeper.sleep_for_us(500'000);
    }
    return 0;
}
