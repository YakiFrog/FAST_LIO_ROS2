#include <rclcpp/rclcpp.hpp>
#include "../src/IMU_Processing.hpp"
#include <iostream>
#include <stdexcept>

int main() {
    ImuProcess processor;
    esekfom::esekf<state_ikfom, 12, input_ikfom> filter;
    MeasureGroup measurement;
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    cloud->resize(1);
    cloud->points[0].x = 123.0;
    processor.Process(measurement, filter, cloud);
    if (!cloud->empty()) throw std::runtime_error("Empty IMU interval reused previous cloud");
    sensor_msgs::msg::Imu::SharedPtr imu(new sensor_msgs::msg::Imu());
    imu->linear_acceleration.z = 9.809;
    imu->header.stamp.sec = 1;
    measurement.imu.push_back(imu);
    cloud->resize(1);
    cloud->points[0].x = 123.0;
    processor.Process(measurement, filter, cloud);
    if (!cloud->empty()) throw std::runtime_error("IMU initialization reused previous cloud");
    std::cout << "PASS: failed/initializing IMU propagation cannot reuse a stale cloud\n";
}
