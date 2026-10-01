#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace hydrox::sitl
{
    /** Replace destination with source without exposing a missing-file window. */
    bool replace_file_atomically(
        const std::string &source,
        const std::string &destination);

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
        bool bind_local(const std::string &host, uint16_t port);
        bool send(const void *data, std::size_t size) const;
        int receive(void *data, std::size_t capacity);
        void accept_last_peer();
        bool has_peer() const;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };

} // namespace hydrox::sitl
