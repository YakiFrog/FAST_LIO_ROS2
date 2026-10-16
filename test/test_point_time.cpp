#include "point_time.hpp"
#include <vector>
#include <stdexcept>
#include <iostream>
struct Point { float curvature; int id; };
void check(bool ok) { if (!ok) throw std::runtime_error("Point acquisition time changed"); }
int main() {
    std::vector<Point> points{{1.3f, 0}, {-49.1f, 1}, {-20.f, 2}};
    const double header = 100.;
    const double start = header + fast_lio::normalize_point_times(points);
    check(std::abs(start-99.9509) < 1e-7);
    const double original[] = {1.3, -49.1, -20.};
    for (const auto &p : points) check(std::abs(start+p.curvature/1000.-(header+original[p.id]/1000.)) < 1e-7);
    check(points.front().curvature == 0. && std::abs(points.back().curvature-50.4) < 1e-5);
    std::vector<Point> positive{{100.f, 0}, {5.f, 1}};
    check(fast_lio::normalize_point_times(positive) == 0.);
    check(positive.front().curvature == 5.f && positive.back().curvature == 100.f);
    std::vector<Point> empty;
    check(fast_lio::normalize_point_times(empty) == 0.);
    bool rejected = false;
    try { std::vector<Point> bad{{1000.f, 0}}; fast_lio::normalize_point_times(bad); }
    catch (const std::runtime_error &) { rejected = true; }
    check(rejected);
    std::cout << "PASS: signed and start-stamped scans preserve absolute point times\n";
}
