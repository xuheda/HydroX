#include "dds_cdr_helpers.h"
#include "dds_setpoint_codec.h"
#include "dds_topic_manifest.h"

#include <ucdr/microcdr.h>

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace
{

bool close_to(double a, double b, double eps = 1e-12)
{
    return std::fabs(a - b) <= eps;
}

int fail(const char *msg)
{
    std::printf("FAIL: %s\n", msg);
    return 1;
}

struct SerializedSetpoint
{
    std::vector<uint8_t> bytes;
    std::size_t frame_terminator_offset = 0;
    std::size_t bool_offset = 0;
};

bool serialize_setpoint(SerializedSetpoint &sample,
                        const std::string &frame_id = "world",
                        const std::string &mode = "WAYPOINT_3D",
                        double depth_ref = 4.5)
{
    sample.bytes.assign(1024, 0);
    ucdrBuffer writer;
    ucdr_init_buffer(&writer, sample.bytes.data(), sample.bytes.size());

    const int32_t sec = 123;
    const uint32_t nanosec = 456'000'000U;
    if (!ucdr_serialize_int32_t(&writer, sec) ||
        !ucdr_serialize_uint32_t(&writer, nanosec))
    {
        return false;
    }

    const std::size_t frame_start = ucdr_buffer_length(&writer);
    if (!ucdr_serialize_string(&writer, frame_id.c_str()))
        return false;
    sample.frame_terminator_offset = frame_start + sizeof(uint32_t) + frame_id.size();

    if (!ucdr_serialize_string(&writer, mode.c_str()) ||
        !ucdr_serialize_double(&writer, depth_ref) ||
        !ucdr_serialize_double(&writer, 1.25) ||
        !ucdr_serialize_double(&writer, 0.75))
    {
        return false;
    }

    sample.bool_offset = ucdr_buffer_length(&writer);
    if (!ucdr_serialize_bool(&writer, true) ||
        !ucdr_serialize_double(&writer, 0.15) ||
        !ucdr_serialize_double(&writer, 10.0) ||
        !ucdr_serialize_double(&writer, 20.0) ||
        !ucdr_serialize_double(&writer, -5.0) ||
        ucdr_buffer_has_error(&writer))
    {
        return false;
    }

    sample.bytes.resize(ucdr_buffer_length(&writer));
    return true;
}

int test_setpoint_decoder()
{
    SerializedSetpoint valid;
    if (!serialize_setpoint(valid))
        return fail("serialize valid setpoint");

    hydrox::GNCSetpointDds decoded;
    std::string error;
    if (!hydrox::dds_cdr::decode_gnc_setpoint(
            valid.bytes.data(), valid.bytes.size(), decoded, &error))
    {
        return fail(error.c_str());
    }
    if (!decoded.valid || decoded.mode != 2 ||
        !decoded.use_yaw_rate_ref || !close_to(decoded.depth_ref, 4.5) ||
        !close_to(decoded.heading_ref, 1.25) ||
        !close_to(decoded.wp_d, -5.0))
    {
        return fail("decoded setpoint values");
    }

    // Every strict prefix of an otherwise valid sample must be rejected. This
    // exercises truncation at length fields, alignment padding and every value.
    for (std::size_t len = 0; len < valid.bytes.size(); ++len)
    {
        hydrox::GNCSetpointDds unchanged;
        unchanged.depth_ref = 321.0;
        unchanged.valid = true;
        if (hydrox::dds_cdr::decode_gnc_setpoint(
                valid.bytes.data(), len, unchanged, nullptr))
        {
            return fail("truncated setpoint accepted");
        }
        if (!unchanged.valid || !close_to(unchanged.depth_ref, 321.0))
            return fail("failed decode modified output");
    }

    SerializedSetpoint oversized_frame;
    if (!serialize_setpoint(
            oversized_frame,
            std::string(hydrox::dds_cdr::MAX_SETPOINT_FRAME_ID_CHARS + 1, 'f')) ||
        hydrox::dds_cdr::decode_gnc_setpoint(
            oversized_frame.bytes.data(), oversized_frame.bytes.size(), decoded, nullptr))
    {
        return fail("oversized frame_id accepted");
    }

    SerializedSetpoint oversized_mode;
    if (!serialize_setpoint(
            oversized_mode,
            "world",
            std::string(hydrox::dds_cdr::MAX_SETPOINT_MODE_CHARS + 1, 'm')) ||
        hydrox::dds_cdr::decode_gnc_setpoint(
            oversized_mode.bytes.data(), oversized_mode.bytes.size(), decoded, nullptr))
    {
        return fail("oversized mode accepted");
    }

    SerializedSetpoint missing_terminator = valid;
    missing_terminator.bytes[missing_terminator.frame_terminator_offset] = 'x';
    if (hydrox::dds_cdr::decode_gnc_setpoint(
            missing_terminator.bytes.data(), missing_terminator.bytes.size(), decoded, nullptr))
    {
        return fail("unterminated frame_id accepted");
    }

    SerializedSetpoint invalid_bool = valid;
    invalid_bool.bytes[invalid_bool.bool_offset] = 2;
    if (hydrox::dds_cdr::decode_gnc_setpoint(
            invalid_bool.bytes.data(), invalid_bool.bytes.size(), decoded, nullptr))
    {
        return fail("invalid CDR bool accepted");
    }

    SerializedSetpoint non_finite;
    if (!serialize_setpoint(
            non_finite,
            "world",
            "WAYPOINT_3D",
            std::numeric_limits<double>::quiet_NaN()) ||
        hydrox::dds_cdr::decode_gnc_setpoint(
            non_finite.bytes.data(), non_finite.bytes.size(), decoded, nullptr))
    {
        return fail("non-finite setpoint accepted");
    }

    SerializedSetpoint unknown_mode;
    if (!serialize_setpoint(unknown_mode, "world", "NOT_A_MODE") ||
        hydrox::dds_cdr::decode_gnc_setpoint(
            unknown_mode.bytes.data(), unknown_mode.bytes.size(), decoded, nullptr))
    {
        return fail("unknown mode accepted");
    }

    std::vector<uint8_t> trailing = valid.bytes;
    trailing.push_back(0);
    if (hydrox::dds_cdr::decode_gnc_setpoint(
            trailing.data(), trailing.size(), decoded, nullptr))
    {
        return fail("trailing CDR data accepted");
    }

    return 0;
}

} // namespace

int main()
{
    static_assert(hydrox::dds_topics::period_us(hydrox::dds_topics::kVehicleStatus) == 1'000'000ULL);
    static_assert(hydrox::dds_topics::period_us(hydrox::dds_topics::kOdometry) == 50'000ULL);
    static_assert(hydrox::dds_topics::kTf.topic_id != hydrox::dds_topics::kSetpoint.topic_id);

    if (hydrox::dds_topics::dds_name(hydrox::dds_topics::kOdometry, "auv0") !=
        std::string("rt/hydrox/auv0/out/odom"))
        return fail("odom dds name");
    if (hydrox::dds_topics::ros_name(hydrox::dds_topics::kSetpoint, "auv0") !=
        std::string("/hydrox/auv0/in/setpoint"))
        return fail("setpoint ros name");
    if (hydrox::dds_topics::dds_name(hydrox::dds_topics::kTf, "auv0") !=
        std::string("rt/tf"))
        return fail("tf dds name");

    uint8_t buffer[1024] = {};
    ucdrBuffer writer;
    ucdr_init_buffer(&writer, buffer, sizeof(buffer));

    constexpr uint64_t stamp_ns = 1'234'567'890'123ULL;
    if (!hydrox::dds_cdr::write_header(&writer, stamp_ns, "world"))
        return fail("write_header");

    constexpr double pi = 3.14159265358979323846;
    if (!hydrox::dds_cdr::write_quaternion_rpy(&writer, 0.0, 0.0, pi * 0.5))
        return fail("write_quaternion_rpy");

    double cov[36] = {};
    for (int i = 0; i < 36; ++i)
        cov[i] = static_cast<double>(i) + 0.25;
    if (!hydrox::dds_cdr::write_covariance_36(&writer, cov))
        return fail("write_covariance_36");

    if (ucdr_buffer_has_error(&writer))
        return fail("writer error");

    ucdrBuffer reader;
    ucdr_init_buffer(&reader, buffer, ucdr_buffer_length(&writer));

    int32_t sec = 0;
    uint32_t nsec = 0;
    char frame_id[16] = {};
    if (!ucdr_deserialize_int32_t(&reader, &sec) ||
        !ucdr_deserialize_uint32_t(&reader, &nsec) ||
        !ucdr_deserialize_string(&reader, frame_id, sizeof(frame_id)))
        return fail("read header");

    if (sec != 1234 || nsec != 567890123 || std::strcmp(frame_id, "world") != 0)
        return fail("header values");

    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0;
    if (!ucdr_deserialize_double(&reader, &qx) ||
        !ucdr_deserialize_double(&reader, &qy) ||
        !ucdr_deserialize_double(&reader, &qz) ||
        !ucdr_deserialize_double(&reader, &qw))
        return fail("read quaternion");

    const double s = std::sqrt(0.5);
    if (!close_to(qx, 0.0) || !close_to(qy, 0.0) ||
        !close_to(qz, s) || !close_to(qw, s))
        return fail("quaternion values");

    double out_cov[36] = {};
    if (!ucdr_deserialize_array_double(&reader, out_cov, 36))
        return fail("read covariance");

    for (int i = 0; i < 36; ++i)
    {
        if (!close_to(out_cov[i], cov[i]))
            return fail("covariance values");
    }

    if (ucdr_buffer_has_error(&reader))
        return fail("reader error");

    if (const int rc = test_setpoint_decoder(); rc != 0)
        return rc;

    return 0;
}
