// geo_utm.h — WGS84 -> UTM, for field boundaries given as GPS coordinates.
//
// The SAME projection the bridge uses for GPS.getPos() (webots-bridge SpecificWorker::wgs84_to_utm,
// Snyder eqs. 8-9..8-10), because Webots itself places its WGS84 output in UTM around the world's
// gpsReference: local = UTM(lat, lon) - UTM(reference), verified to < 1 mm (plan.md F1). Converting a
// boundary any other way (a tangent plane, say) would put it 12 cm off at 5 m from the origin.
#pragma once

#include <cmath>

namespace rc::openfield
{

struct Utm { int zone = 0; double east = 0, north = 0; };

// force_zone > 0 keeps a point in the reference's zone even across a boundary, so the local frame
// never jumps by a zone width.
inline Utm wgs84_to_utm(double lat_deg, double lon_deg, int force_zone = 0)
{
    constexpr double a = 6378137.0, f = 1.0 / 298.257223563, k0 = 0.9996;
    constexpr double e2 = f * (2.0 - f), ep2 = e2 / (1.0 - e2);
    Utm u;
    u.zone = force_zone > 0 ? force_zone : static_cast<int>(std::floor((lon_deg + 180.0) / 6.0)) + 1;
    const double lon0 = ((u.zone - 1) * 6 - 180 + 3) * M_PI / 180.0;
    const double phi = lat_deg * M_PI / 180.0, lam = lon_deg * M_PI / 180.0;
    const double s = std::sin(phi), c = std::cos(phi), t = std::tan(phi);
    const double N = a / std::sqrt(1.0 - e2 * s * s);
    const double T = t * t, C = ep2 * c * c, A = c * (lam - lon0);
    const double M = a * ((1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256) * phi
                        - (3 * e2 / 8 + 3 * e2 * e2 / 32 + 45 * e2 * e2 * e2 / 1024) * std::sin(2 * phi)
                        + (15 * e2 * e2 / 256 + 45 * e2 * e2 * e2 / 1024) * std::sin(4 * phi)
                        - (35 * e2 * e2 * e2 / 3072) * std::sin(6 * phi));
    u.east  = k0 * N * (A + (1 - T + C) * std::pow(A, 3) / 6
                        + (5 - 18 * T + T * T + 72 * C - 58 * ep2) * std::pow(A, 5) / 120) + 500000.0;
    u.north = k0 * (M + N * t * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * std::pow(A, 4) / 24
                                 + (61 - 58 * T + T * T + 600 * C - 330 * ep2) * std::pow(A, 6) / 720));
    if (lat_deg < 0) u.north += 10000000.0;
    return u;
}

} // namespace rc::openfield
