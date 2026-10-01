// Copyright (c) 2026 OceanX
#include "geodesy.h"

#include <geodesic.h>

#include <cmath>

namespace hydrox::geodesy
{
    namespace
    {
        constexpr double kWgs84SemiMajorAxisM = 6378137.0;
        constexpr double kWgs84Flattening = 1.0 / 298.257223563;
        constexpr double kDegreesToRadians = 0.017453292519943295769;
        constexpr double kRadiansToDegrees = 57.295779513082320877;

        const geod_geodesic &wgs84()
        {
            // C++11 guarantees thread-safe initialization. GeographicLib-C's
            // geod_geodesic value is immutable after geod_init.
            static const geod_geodesic model = []
            {
                geod_geodesic value{};
                geod_init(&value, kWgs84SemiMajorAxisM, kWgs84Flattening);
                return value;
            }();
            return model;
        }

        double normalize_longitude_deg(double longitude_deg)
        {
            double wrapped = std::fmod(longitude_deg + 180.0, 360.0);
            if (wrapped < 0.0)
                wrapped += 360.0;
            return wrapped - 180.0;
        }

        bool valid_geodetic(double latitude_deg, double longitude_deg, double altitude_m)
        {
            return std::isfinite(latitude_deg) &&
                   std::isfinite(longitude_deg) &&
                   std::isfinite(altitude_m) &&
                   latitude_deg >= -90.0 && latitude_deg <= 90.0 &&
                   longitude_deg >= -180.0 && longitude_deg <= 180.0;
        }
    } // namespace

    bool is_valid(const Wgs84AeqdFrame &frame)
    {
        return valid_geodetic(
                   frame.origin_latitude_deg,
                   frame.origin_longitude_deg,
                   frame.origin_altitude_m) &&
               std::isfinite(frame.max_horizontal_radius_m) &&
               frame.max_horizontal_radius_m > 0.0;
    }

    bool geodetic_to_local_ned(
        const Wgs84AeqdFrame &frame,
        double latitude_deg,
        double longitude_deg,
        double altitude_m,
        double &north_m,
        double &east_m,
        double &down_m)
    {
        if (!is_valid(frame) ||
            !valid_geodetic(latitude_deg, longitude_deg, altitude_m))
        {
            return false;
        }

        double distance_m = 0.0;
        double azimuth_deg = 0.0;
        geod_inverse(
            &wgs84(),
            frame.origin_latitude_deg,
            frame.origin_longitude_deg,
            latitude_deg,
            longitude_deg,
            &distance_m,
            &azimuth_deg,
            nullptr);
        if (!std::isfinite(distance_m) ||
            !std::isfinite(azimuth_deg) ||
            distance_m > frame.max_horizontal_radius_m)
        {
            return false;
        }

        const double azimuth_rad = azimuth_deg * kDegreesToRadians;
        const double result_north_m = distance_m * std::cos(azimuth_rad);
        const double result_east_m = distance_m * std::sin(azimuth_rad);
        const double result_down_m = frame.origin_altitude_m - altitude_m;
        if (!std::isfinite(result_north_m) ||
            !std::isfinite(result_east_m) ||
            !std::isfinite(result_down_m))
        {
            return false;
        }

        north_m = result_north_m;
        east_m = result_east_m;
        down_m = result_down_m;
        return true;
    }

    bool local_ned_to_geodetic(
        const Wgs84AeqdFrame &frame,
        double north_m,
        double east_m,
        double down_m,
        double &latitude_deg,
        double &longitude_deg,
        double &altitude_m)
    {
        if (!is_valid(frame) ||
            !std::isfinite(north_m) ||
            !std::isfinite(east_m) ||
            !std::isfinite(down_m))
        {
            return false;
        }

        const double distance_m = std::hypot(north_m, east_m);
        if (!std::isfinite(distance_m) ||
            distance_m > frame.max_horizontal_radius_m)
        {
            return false;
        }

        const double azimuth_deg =
            std::atan2(east_m, north_m) * kRadiansToDegrees;
        double result_latitude_deg = 0.0;
        double result_longitude_deg = 0.0;
        geod_direct(
            &wgs84(),
            frame.origin_latitude_deg,
            frame.origin_longitude_deg,
            azimuth_deg,
            distance_m,
            &result_latitude_deg,
            &result_longitude_deg,
            nullptr);
        result_longitude_deg = normalize_longitude_deg(result_longitude_deg);
        const double result_altitude_m = frame.origin_altitude_m - down_m;
        if (!valid_geodetic(
                result_latitude_deg,
                result_longitude_deg,
                result_altitude_m))
        {
            return false;
        }

        latitude_deg = result_latitude_deg;
        longitude_deg = result_longitude_deg;
        altitude_m = result_altitude_m;
        return true;
    }
} // namespace hydrox::geodesy
