#include "dds_cdr_writer.h"
#include "frame_contract.h"
#include "odometry_contract.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>

namespace
{
    bool approx(double lhs, double rhs, double tolerance = 1e-12)
    {
        return std::abs(lhs - rhs) <= tolerance;
    }

    struct TestCovariance
    {
        double values[12][12]{};

        double operator()(int row, int column) const
        {
            return values[row][column];
        }
    };

    int fail(const char *message)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        return 1;
    }
}

int main()
{
    if (std::strcmp(hydrox::frame_contract::kMapNed, "map_ned") != 0 ||
        hydrox::frame_contract::body_frd("vehicle0") != "vehicle0/base_link_frd" ||
        hydrox::frame_contract::sensor_frd("vehicle0", "imu") != "vehicle0/imu_frd" ||
        !hydrox::frame_contract::is_valid_frame_token("VEHICLE_01") ||
        hydrox::frame_contract::is_valid_frame_token("vehicle/01") ||
        !hydrox::frame_contract::body_frd("bad name").empty())
    {
        return fail("canonical coordinate frame ids changed");
    }

    std::array<uint8_t, 128> storage{};
    hydrox::dds_cdr::Writer writer(storage.data(), storage.size());
    writer.header(1'500'000'123ULL, hydrox::frame_contract::kMapNed);
    writer.string("vehicle0");
    writer.uint32(42);
    writer.float64(3.25);
    writer.boolean(true);

    if (!writer.ok() || writer.size() == 0 || writer.size() >= storage.size())
        return fail("valid CDR payload was not finalized with an exact length");

    ucdrBuffer reader;
    ucdr_init_buffer(&reader, storage.data(), writer.size());
    int32_t sec = 0;
    uint32_t nanosec = 0;
    char frame[16]{};
    char vehicle[16]{};
    uint32_t number = 0;
    double value = 0.0;
    bool enabled = false;
    const bool decoded =
        ucdr_deserialize_int32_t(&reader, &sec) &&
        ucdr_deserialize_uint32_t(&reader, &nanosec) &&
        ucdr_deserialize_string(&reader, frame, sizeof(frame)) &&
        ucdr_deserialize_string(&reader, vehicle, sizeof(vehicle)) &&
        ucdr_deserialize_uint32_t(&reader, &number) &&
        ucdr_deserialize_double(&reader, &value) &&
        ucdr_deserialize_bool(&reader, &enabled);
    if (!decoded || sec != 1 || nanosec != 500'000'123U ||
        std::strcmp(frame, "map_ned") != 0 ||
        std::strcmp(vehicle, "vehicle0") != 0 || number != 42 ||
        value != 3.25 || !enabled)
    {
        return fail("exact-length payload did not deserialize correctly");
    }
    if (ucdr_buffer_remaining(&reader) != 0)
        return fail("exact-length payload contained trailing bytes");

    std::array<uint8_t, 8> tiny_storage{};
    hydrox::dds_cdr::Writer tiny(tiny_storage.data(), tiny_storage.size());
    tiny.string("this string cannot fit");
    tiny.uint64(123);
    if (tiny.ok() || tiny.size() != 0)
        return fail("overflowed payload was accepted");

    hydrox::dds_cdr::Writer invalid(nullptr, 0);
    invalid.uint32(1);
    if (invalid.ok() || invalid.size() != 0)
        return fail("invalid backing storage was accepted");

    // Lock the shared capacity against the largest current schema: Odometry
    // contains pose and twist covariance arrays (72 doubles total).
    std::array<uint8_t, hydrox::dds_cdr::MAX_TOPIC_PAYLOAD_BYTES> odom_storage{};
    hydrox::dds_cdr::Writer odom(odom_storage.data(), odom_storage.size());
    double covariance[36]{};
    odom.header(1'500'000'123ULL, hydrox::frame_contract::kMapNed);
    odom.string(
        hydrox::frame_contract::body_frd(
            "vehicle_with_a_deliberately_long_name").c_str());
    odom.float64(1.0);
    odom.float64(2.0);
    odom.float64(3.0);
    odom.quaternion_rpy(0.1, 0.2, 0.3);
    odom.covariance_36(covariance);
    for (int i = 0; i < 6; ++i)
        odom.float64(static_cast<double>(i));
    odom.covariance_36(covariance);
    if (!odom.ok() || odom.size() >= odom_storage.size())
        return fail("largest current DDS schema exceeds the shared payload bound");

    TestCovariance ekf_covariance;
    for (int axis = 0; axis < 12; ++axis)
        ekf_covariance.values[axis][axis] = axis + 1.0;
    ekf_covariance.values[0][1] = 0.2;
    ekf_covariance.values[1][0] = 0.4;
    ekf_covariance.values[6][7] = 0.6;
    ekf_covariance.values[7][6] = 0.2;

    double pose_covariance[36]{};
    double twist_covariance[36]{};
    if (!hydrox::odometry_contract::encode_covariances(
            ekf_covariance, pose_covariance, twist_covariance, 0.05) ||
        !approx(pose_covariance[1], 0.3) ||
        !approx(pose_covariance[6], 0.3) ||
        !approx(twist_covariance[1], 0.4) ||
        !approx(twist_covariance[6], 0.4) ||
        !approx(twist_covariance[21], 0.05) ||
        !approx(twist_covariance[28], 0.05) ||
        !approx(twist_covariance[35], 0.05))
    {
        return fail("odometry covariance frame/order contract changed");
    }

    ekf_covariance.values[2][2] = -1.0;
    if (hydrox::odometry_contract::encode_covariances(
            ekf_covariance, pose_covariance, twist_covariance))
    {
        return fail("invalid covariance was published");
    }

    ekf_covariance.values[2][2] = 3.0;
    ekf_covariance.values[1][2] =
        std::numeric_limits<double>::quiet_NaN();
    if (hydrox::odometry_contract::encode_covariances(
            ekf_covariance, pose_covariance, twist_covariance))
    {
        return fail("non-finite covariance was published");
    }

    return 0;
}
