#pragma once
/**
 * transport.h — Transport layer abstract interface
 *
 * SITL: TcpTransport  (TCP socket, POSIX / Winsock)
 * HIL : UartTransport (STM32 UART HAL)
 *
 * FCNode only depends on this interface, containing no platform-specific code.
 */
#include <cstddef>
#include <cstdint>
#include <string>

namespace hydrox
{

    class Transport
    {
    public:
        virtual ~Transport() = default;

        /**
         * Blocking connect (TCP: wait for client; UART: open serial port).
         * Returns true on success.
         */
        virtual bool connect() = 0;

        /**
         * Disconnect and release resources.
         */
        virtual void disconnect() = 0;

        /**
         * Non-blocking read (read up to len bytes).
         * @return Number of bytes actually read; 0 means no data available, -1 means connection lost.
         */
        virtual int read(uint8_t *buf, size_t len) = 0;

        /**
         * Submit one complete transport frame.
         *
         * A non-blocking stream implementation may retain the tail of a frame
         * that was only partially written. It must never insert another frame
         * before that tail or silently discard a tail already on the stream.
         * @return true while the connection remains usable, false if lost.
         */
        virtual bool write(const uint8_t *buf, size_t len) = 0;

        /** Whether the connection is valid */
        virtual bool is_connected() const = 0;
    };

} // namespace hydrox
