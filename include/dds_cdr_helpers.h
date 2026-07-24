#pragma once

#include <ucdr/microcdr.h>

#include <cmath>
#include <cstdint>
#include <cstring>

namespace hydrox::dds_cdr
{

inline bool write_string(ucdrBuffer *ub, const char *value)
{
    return ucdr_serialize_string(ub, value ? value : "");
}

inline bool write_header(ucdrBuffer *ub, uint64_t stamp_ns,
                         const char *frame_id = "")
{
    const int32_t sec = static_cast<int32_t>(stamp_ns / 1'000'000'000ULL);
    const uint32_t nsec = static_cast<uint32_t>(stamp_ns % 1'000'000'000ULL);
    return ucdr_serialize_int32_t(ub, sec) &&
           ucdr_serialize_uint32_t(ub, nsec) &&
           write_string(ub, frame_id);
}

inline bool write_quaternion_rpy(ucdrBuffer *ub, double roll,
                                 double pitch, double yaw)
{
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);

    const double qx = sr * cp * cy - cr * sp * sy;
    const double qy = cr * sp * cy + sr * cp * sy;
    const double qz = cr * cp * sy - sr * sp * cy;
    const double qw = cr * cp * cy + sr * sp * sy;

    return ucdr_serialize_double(ub, qx) &&
           ucdr_serialize_double(ub, qy) &&
           ucdr_serialize_double(ub, qz) &&
           ucdr_serialize_double(ub, qw);
}

inline bool write_covariance_36(ucdrBuffer *ub, const double cov[36])
{
    return ucdr_serialize_array_double(ub, cov, 36);
}

} // namespace hydrox::dds_cdr
