#pragma once

#include "dds_cdr_helpers.h"

#include <cstddef>
#include <cstdint>

namespace hydrox::dds_cdr
{
    inline constexpr size_t MAX_TOPIC_PAYLOAD_BYTES = 2048;

    // Accumulates every Micro-CDR return value so call sites cannot silently
    // continue after a serialization failure. The writer does not own memory.
    class Writer
    {
    public:
        Writer(uint8_t *data, size_t capacity)
            : data_(data)
        {
            if (data_ == nullptr || capacity == 0)
            {
                ok_ = false;
                return;
            }
            ucdr_init_buffer(&buffer_, data_, capacity);
        }

        Writer(const Writer &) = delete;
        Writer &operator=(const Writer &) = delete;

        bool ok() const
        {
            return ok_ && !ucdr_buffer_has_error(&buffer_);
        }

        size_t size() const
        {
            return ok() ? ucdr_buffer_length(&buffer_) : 0;
        }

        uint8_t *data()
        {
            return data_;
        }

        size_t remaining() const
        {
            return ok() ? ucdr_buffer_remaining(&buffer_) : 0;
        }

        void header(uint64_t stamp_ns, const char *frame_id = "")
        {
            if (ok_)
                ok_ = dds_cdr::write_header(&buffer_, stamp_ns, frame_id);
        }

        void string(const char *value)
        {
            if (ok_)
                ok_ = dds_cdr::write_string(&buffer_, value);
        }

        void boolean(bool value)
        {
            if (ok_)
                ok_ = ucdr_serialize_bool(&buffer_, value);
        }

        void uint8(uint8_t value)
        {
            if (ok_)
                ok_ = ucdr_serialize_uint8_t(&buffer_, value);
        }

        void uint32(uint32_t value)
        {
            if (ok_)
                ok_ = ucdr_serialize_uint32_t(&buffer_, value);
        }

        void uint64(uint64_t value)
        {
            if (ok_)
                ok_ = ucdr_serialize_uint64_t(&buffer_, value);
        }

        void int32(int32_t value)
        {
            if (ok_)
                ok_ = ucdr_serialize_int32_t(&buffer_, value);
        }

        void float32(float value)
        {
            if (ok_)
                ok_ = ucdr_serialize_float(&buffer_, value);
        }

        void float64(double value)
        {
            if (ok_)
                ok_ = ucdr_serialize_double(&buffer_, value);
        }

        void float32_array(const float *values, size_t count)
        {
            if (ok_)
                ok_ = ucdr_serialize_array_float(&buffer_, values, count);
        }

        void float64_array(const double *values, size_t count)
        {
            if (ok_)
                ok_ = ucdr_serialize_array_double(&buffer_, values, count);
        }

        void quaternion_rpy(double roll, double pitch, double yaw)
        {
            if (ok_)
            {
                ok_ = dds_cdr::write_quaternion_rpy(
                    &buffer_, roll, pitch, yaw);
            }
        }

        void covariance_36(const double values[36])
        {
            if (ok_)
                ok_ = dds_cdr::write_covariance_36(&buffer_, values);
        }

    private:
        uint8_t *data_ = nullptr;
        ucdrBuffer buffer_{};
        bool ok_ = true;
    };
} // namespace hydrox::dds_cdr
