#ifndef FAST_LIO_POINT_TIME_HPP
#define FAST_LIO_POINT_TIME_HPP
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fast_lio {
// curvature stores point milliseconds relative to the driver header. Some
// Velodyne drivers stamp the last packet, so offsets are legitimately negative.
// Move BOTH the scan origin and point offsets, preserving every acquisition time.
template<class Points>
double normalize_point_times(Points &points) {
    if (points.empty()) return 0.;
    double earliest = 0., latest = 0.;
    for (const auto &point : points) {
        if (!std::isfinite(point.curvature)) throw std::runtime_error("Nonfinite LiDAR point time");
        earliest = std::min(earliest, double(point.curvature));
        latest = std::max(latest, double(point.curvature));
    }
    if (latest-earliest > 500.) throw std::runtime_error("LiDAR point time span exceeds 0.5 seconds; check timestamp_unit");
    if (earliest < 0.) for (auto &point : points) point.curvature -= earliest;
    std::stable_sort(points.begin(), points.end(), [](const auto &a, const auto &b) {
        return a.curvature < b.curvature;
    });
    return earliest/1000.;
}
}
#endif
