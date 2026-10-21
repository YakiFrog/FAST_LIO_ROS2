#include "../src/preprocess.h"
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

int main() {
    Preprocess processor;
    processor.lidar_type = VELO16;
    processor.N_SCANS = 16;
    processor.point_filter_num = 4;
    processor.time_unit = SEC;
    processor.blind = .1;
    // Reproduce the actual driver order: rings 0..15 at each azimuth.
    pcl::PointCloud<velodyne_ros::Point> input;
    for (int azimuth = 0; azimuth < 40; ++azimuth) {
        for (int ring = 0; ring < 16; ++ring) {
            velodyne_ros::Point p{};
            p.x = 3.f;
            p.y = float(azimuth) * .1f;
            p.z = float(ring) * .1f;
            p.intensity = float(ring);
            p.ring = ring;
            p.time = -.05f + azimuth * .001f;
            input.push_back(p);
        }
    }
    sensor_msgs::msg::PointCloud2::UniquePtr msg(new sensor_msgs::msg::PointCloud2());
    pcl::toROSMsg(input, *msg);
    PointCloudXYZI::Ptr output(new PointCloudXYZI());
    processor.process(msg, output);
    std::array<int, 16> counts{};
    if (output->size() != 160) throw std::runtime_error("Unexpected sampling density");
    for (const auto &p : *output) {
        const int ring = std::lround(p.intensity);
        counts.at(ring)++;
        const int azimuth = std::lround(p.y * 10.f);
        if (std::abs(p.curvature - 1000.f * (-.05f + azimuth * .001f)) > 1e-4)
            throw std::runtime_error("Sampling changed a point acquisition time");
    }
    for (int n : counts) if (n != 10) throw std::runtime_error("Vertical ring was lost");
    std::cout << "PASS: real Velodyne preprocessing retains all 16 rings and original timestamps\n";
}
