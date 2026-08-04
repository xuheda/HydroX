// Copyright (c) 2026 OceanX
#pragma once

namespace hydrox::geodesy
{
    /**
     * OceanX geodesy contract v1.
     *
     * Horizontal coordinates use the WGS-84 azimuthal-equidistant projection
     * centred on the immutable mission origin. Vertical coordinates are kept
     * independent: down = origin altitude - point altitude. The caller must
     * use the same vertical datum for both altitudes (MSL for MAVLink HIL_GPS).
     */
    inline constexpr unsigned kContractVersion = 1;
    inline constexpr const char *kProjectionId = "wgs84-aeqd-v1";

    struct Wgs84AeqdFrame
    {
        double origin_latitude_deg = 0.0;
        double origin_longitude_deg = 0.0;
        double origin_altitude_m = 0.0;
        double max_horizontal_radius_m = 10000.0;
    };

    bool is_valid(const Wgs84AeqdFrame &frame);

    /** WGS-84 geodetic coordinate to local NED metres. */
    bool geodetic_to_local_ned(
        const Wgs84AeqdFrame &frame,
        double latitude_deg,
        double longitude_deg,
        double altitude_m,
        double &north_m,
        double &east_m,
        double &down_m);

    /** Local NED metres to WGS-84 geodetic coordinate. */
    bool local_ned_to_geodetic(
        const Wgs84AeqdFrame &frame,
        double north_m,
        double east_m,
        double down_m,
        double &latitude_deg,
        double &longitude_deg,
        double &altitude_m);
} // namespace hydrox::geodesy
