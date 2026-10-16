#include "use-ikfom.hpp"
#include "wheel_odometry.hpp"
#include "scan_imu_sync.hpp"
#include <Eigen/Geometry>
#include <iostream>
#include <stdexcept>

using Filter = esekfom::esekf<state_ikfom, 12, input_ikfom>;

void require(bool value, const char *message) {
    if (!value) throw std::runtime_error(message);
}

bool with_wheel = false;
int wall_rows = 2, columns = 12;
fast_lio::WheelSample wheel;
fast_lio::WheelConfig config;

void wall_measurement(state_ikfom &state, esekfom::dyn_share_datastruct<double> &data) {
    // A single straight wall and floor do not observe motion along the x axis.
    data.h_x = Eigen::MatrixXd::Zero(wall_rows + (with_wheel ? 4 : 0), columns);
    data.h = Eigen::VectorXd::Zero(data.h_x.rows());
    for (int i = 0; i < wall_rows; ++i) {
        const int axis = 1 + i % 2;
        data.h_x(i, axis) = 1.0;
        data.h(i) = -state.pos(axis);
    }
    if (with_wheel) {
        const auto obs = fast_lio::wheel_observation(state.rot.toRotationMatrix(),
            state.vel, state.bg, Eigen::Vector3d::Zero(), wheel, config, 0.001);
        data.h_x.bottomRows(4) = obs.jacobian;
        data.h.tail(4) = obs.residual;
    }
}

state_ikfom drive(bool enabled, int rows, int cols) {
    with_wheel = enabled; wall_rows = rows; columns = cols;
    Filter filter;
    double limits[23];
    std::fill(limits, limits + 23, 1e-6);
    filter.init_dyn_share(get_f, df_dx, df_dw, wall_measurement, 3, limits);
    state_ikfom state;
    state.grav = S2(0.0, 0.0, -9.809);
    state.vel << 1.0, 0.0, 0.0;
    filter.change_x(state);
    Filter::cov covariance = Filter::cov::Identity() * 0.1;
    filter.change_P(covariance);
    wheel.velocity << 1.0, 0.0, 0.0;
    wheel.variance.setConstant(0.01);
    auto noise = process_noise_cov();
    for (int i = 0; i < 100; ++i) {
        input_ikfom input;
        input.acc << 0.0, 0.0, 9.809;
        input.acc.x() = 0.2; // Unmodelled IMU bias; true speed remains 1 m/s.
        input.gyro.setZero();
        double dt = 0.1;
        filter.predict(dt, noise, input);
        double solve = 0.0;
        filter.update_iterated_dyn_share_modified(0.001, solve);
        require(filter.get_P().allFinite(), "Nonfinite covariance");
    }
    const auto p = filter.get_P();
    require((p - p.transpose()).norm() < 1e-6, "Asymmetric covariance");
    require(Eigen::SelfAdjointEigenSolver<Filter::cov>(p).eigenvalues().minCoeff() > -1e-8,
            "Covariance lost positive semidefiniteness");
    return filter.get_x();
}

void test_buffer() {
    fast_lio::WheelBuffer buffer;
    fast_lio::WheelSample sample, result;
    sample.stamp = 1.0; sample.velocity.x() = 1.0;
    require(buffer.push(sample), "Valid wheel sample rejected");
    sample.stamp = 1.1; sample.velocity.x() = 3.0;
    require(buffer.push(sample), "Second wheel sample rejected");
    require(buffer.at(1.05, 0.2, result) && std::abs(result.velocity.x() - 2.0) < 1e-9,
            "Wheel sample was not interpolated at scan time");
    require(!buffer.at(1.5, 0.2, result), "Stale wheel data used");
    require(!buffer.at(0.5, 0.2, result), "Distant future wheel data used");
    sample.stamp = 1.01;
    require(!buffer.push(sample), "Out-of-order sample accepted");
    sample.stamp = 0.0;
    require(buffer.push(sample) && !buffer.at(1.1, 0.2, result), "Time reset retained old data");
    sample.velocity.x() = std::numeric_limits<double>::quiet_NaN();
    require(!buffer.push(sample), "NaN wheel data accepted");
}

void test_equal_rate_imu_sync() {
    std::deque<double> buffered, selected;
    for (int scan = 1; scan <= 100; ++scan) {
        const double stamp = scan * 0.1;
        buffered.push_back(stamp);
        fast_lio::collect_scan_imu(buffered, stamp, selected, [](double t) { return t; });
        require(selected.size() == 1 && selected.front() == stamp,
                "An IMU at exactly the scan stamp was lost");
        require(buffered.empty(), "Equal-stamp IMU leaked into the next scan");
    }
    buffered = {10.1, 10.2, 10.3};
    fast_lio::collect_scan_imu(buffered, 10.2, selected, [](double t) { return t; });
    require(selected.size() == 2 && buffered.size() == 1 && buffered.front() == 10.3,
            "IMU collection consumed future samples or lost the endpoint");
    fast_lio::collect_scan_imu(buffered, 10.25, selected, [](double t) { return t; });
    require(selected.empty() && buffered.size() == 1, "Empty interval reused prior IMUs");
}

void test_jacobian() {
    fast_lio::WheelConfig calibration;
    calibration.imu_from_base = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    calibration.imu_to_base << 0.1, -0.2, -0.3;
    calibration.velocity_std = calibration.yaw_rate_std = 1.0;
    Eigen::Matrix3d rotation = Eigen::AngleAxisd(0.4, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
    Eigen::Vector3d velocity(1, 2, 3), bias(0.01, 0.02, 0.03), gyro(0.2, 0.3, 0.4);
    fast_lio::WheelSample sample;
    sample.variance.setOnes();
    auto base = fast_lio::wheel_observation(rotation, velocity, bias, gyro, sample, calibration, 1.0);
    for (int axis = 0; axis < 3; ++axis) {
        const Eigen::Vector3d d = Eigen::Vector3d::Unit(axis) * 1e-7;
        auto r = fast_lio::wheel_observation(rotation * Eigen::AngleAxisd(1e-7, Eigen::Vector3d::Unit(axis)).toRotationMatrix(),
            velocity, bias, gyro, sample, calibration, 1.0);
        auto v = fast_lio::wheel_observation(rotation, velocity+d, bias, gyro, sample, calibration, 1.0);
        auto b = fast_lio::wheel_observation(rotation, velocity, bias+d, gyro, sample, calibration, 1.0);
        require(((r.residual-base.residual)/1e-7 + base.jacobian.col(3+axis)).norm() < 1e-6,
                "Rotation Jacobian incorrect");
        require(((v.residual-base.residual)/1e-7 + base.jacobian.col(12+axis)).norm() < 1e-6,
                "Velocity Jacobian incorrect");
        require(((b.residual-base.residual)/1e-7 + base.jacobian.col(15+axis)).norm() < 1e-6,
                "Gyro/lever-arm Jacobian incorrect");
    }
}

int main() {
    try {
        test_buffer();
        test_equal_rate_imu_sync();
        test_jacobian();
        const auto baseline = drive(false, 2, 12);
        for (int rows : {2, 40}) {
            const auto original = drive(false, rows, 12);
            const auto padded = drive(false, rows, 23);
            require((original.pos-padded.pos).norm() < 1e-8, "12-column LiDAR compatibility broken");
        }
        require(std::abs(baseline.pos.x()-10.0) > 5.0, "Degenerate baseline did not drift");
        for (int rows : {0, 2, 40}) {
            const auto fused = drive(true, rows, 23);
            std::cout << "wall rows=" << rows << " straight-line error: LIO="
                      << baseline.pos.x()-10.0 << " m, wheel=" << fused.pos.x()-10.0 << " m\n";
            require(std::abs(fused.pos.x()-10.0) < 0.3, "Wheel fusion failed along unobservable axis");
        }
        std::cout << "PASS: time alignment, Jacobians, both EKF branches, wheel-only update and degeneration\n";
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
