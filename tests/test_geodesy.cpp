// Copyright (c) 2026 OceanX
#include "geodesy.h"

#include <cmath>
#include <cstdio>
#include <limits>

namespace
{
    bool expect(bool condition, const char *message)
    {
        if (!condition)
            std::fprintf(stderr, "FAIL: %s\n", message);
        return condition;
    }

    bool near(double lhs, double rhs, double tolerance)
    {
        return std::abs(lhs - rhs) <= tolerance;
    }
}

int main()
{
    using hydrox::geodesy::Wgs84AeqdFrame;
    using hydrox::geodesy::geodetic_to_local_ned;
    using hydrox::geodesy::is_valid;
    using hydrox::geodesy::local_ned_to_geodetic;

    int failures = 0;
    const Wgs84AeqdFrame shanghai{31.2, 121.5, 8.7, 10000.0};
    failures += expect(is_valid(shanghai), "Shanghai contract is valid") ? 0 : 1;

    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double altitude_m = 0.0;
    failures += expect(
        local_ned_to_geodetic(
            shanghai,
            1000.0,
            -500.0,
            20.0,
            latitude_deg,
            longitude_deg,
            altitude_m),
        "NED to WGS-84 AEQD succeeds") ? 0 : 1;
    failures += expect(
        near(latitude_deg, 31.209019222595042, 1.0e-11) &&
            near(longitude_deg, 121.494753152353042, 1.0e-11) &&
            near(altitude_m, -11.3, 1.0e-12),
        "Shanghai result matches the GeographicLib-C v2.2 golden vector") ? 0 : 1;


    double north_m = 0.0;
    double east_m = 0.0;
    double down_m = 0.0;
    failures += expect(
        geodetic_to_local_ned(
            shanghai,
            latitude_deg,
            longitude_deg,
            altitude_m,
            north_m,
            east_m,
            down_m) &&
            near(north_m, 1000.0, 1.0e-6) &&
            near(east_m, -500.0, 1.0e-6) &&
            near(down_m, 20.0, 1.0e-12),
        "Shanghai AEQD round trip is sub-micrometre") ? 0 : 1;

    const Wgs84AeqdFrame dateline{0.0, 179.999, 0.0, 10000.0};
    failures += expect(
        local_ned_to_geodetic(
            dateline,
            0.0,
            500.0,
            0.0,
            latitude_deg,
            longitude_deg,
            altitude_m) &&
            longitude_deg < -179.0,
        "AEQD wraps eastward across the dateline") ? 0 : 1;
    failures += expect(
        near(latitude_deg, 0.0, 1.0e-12) &&
            near(longitude_deg, -179.996508423579400, 1.0e-11) &&
            near(altitude_m, 0.0, 1.0e-12),
        "Dateline result matches the GeographicLib-C v2.2 golden vector") ? 0 : 1;

    failures += expect(
        geodetic_to_local_ned(
            dateline,
            latitude_deg,
            longitude_deg,
            altitude_m,
            north_m,
            east_m,
            down_m) &&
            near(north_m, 0.0, 1.0e-6) &&
            near(east_m, 500.0, 1.0e-6),
        "Dateline AEQD round trip is sub-micrometre") ? 0 : 1;

    const Wgs84AeqdFrame high_latitude{89.9, 30.0, 100.0, 5000.0};
    failures += expect(
        local_ned_to_geodetic(
            high_latitude,
            707.1067811865476,
            707.1067811865476,
            -5.0,
            latitude_deg,
            longitude_deg,
            altitude_m),
        "AEQD remains valid near the pole") ? 0 : 1;
    failures += expect(
        near(latitude_deg, 89.906117059431409, 1.0e-11) &&
            near(longitude_deg, 33.866527157070948, 1.0e-11) &&
            near(altitude_m, 105.0, 1.0e-12),
        "High-latitude result matches the GeographicLib-C v2.2 golden vector") ? 0 : 1;

    failures += expect(
        geodetic_to_local_ned(
            high_latitude,
            latitude_deg,
            longitude_deg,
            altitude_m,
            north_m,
            east_m,
            down_m) &&
            near(north_m, 707.1067811865476, 1.0e-6) &&
            near(east_m, 707.1067811865476, 1.0e-6) &&
            near(down_m, -5.0, 1.0e-12),
        "High-latitude AEQD round trip is sub-micrometre") ? 0 : 1;

    failures += expect(
        !local_ned_to_geodetic(
            shanghai,
            10000.01,
            0.0,
            0.0,
            latitude_deg,
            longitude_deg,
            altitude_m),
        "NED outside configured radius fails closed") ? 0 : 1;
    failures += expect(
        !geodetic_to_local_ned(
            shanghai,
            31.4,
            121.5,
            8.7,
            north_m,
            east_m,
            down_m),
        "Geodetic point outside configured radius fails closed") ? 0 : 1;

    Wgs84AeqdFrame invalid = shanghai;
    invalid.origin_latitude_deg = std::numeric_limits<double>::quiet_NaN();
    failures += expect(!is_valid(invalid), "NaN origin is rejected") ? 0 : 1;

    if (failures == 0)
        std::printf("test_geodesy: all checks passed\n");
    return failures == 0 ? 0 : 1;
}
