#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace hydrox::sitl
{
    class NetworkRuntime
    {
    public:
        NetworkRuntime();
        ~NetworkRuntime();

        NetworkRuntime(const NetworkRuntime &) = delete;
        NetworkRuntime &operator=(const NetworkRuntime &) = delete;

        bool ready() const;

    private:
        bool ready_{false};
    };

    class ParentProcessGuard
    {
    public:
        explicit ParentProcessGuard(uint64_t parent_pid);
        ~ParentProcessGuard();

        ParentProcessGuard(const ParentProcessGuard &) = delete;
        ParentProcessGuard &operator=(const ParentProcessGuard &) = delete;

        bool arm();
        bool is_parent_alive() const;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

    class UdpSender
    {
    public:
        UdpSender(const std::string &host, uint16_t port, bool broadcast);
        ~UdpSender();

        UdpSender(const UdpSender &) = delete;
        UdpSender &operator=(const UdpSender &) = delete;

        bool is_open() const;
        bool send(const void *data, std::size_t size) const;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

} // namespace hydrox::sitl
