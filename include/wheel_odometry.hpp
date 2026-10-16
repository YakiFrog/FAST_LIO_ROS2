#ifndef FAST_LIO_WHEEL_ODOMETRY_HPP
#define FAST_LIO_WHEEL_ODOMETRY_HPP

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <deque>

namespace fast_lio {

struct WheelSample {
    double stamp = 0.0;
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    double yaw_rate = 0.0;
    Eigen::Vector4d variance = Eigen::Vector4d::Zero();
};

// The odometry pose/origin is deliberately unused: twist is expressed in child_frame_id.
class WheelBuffer {
public:
    bool push(const WheelSample &sample) {
        if (!std::isfinite(sample.stamp) || !sample.velocity.allFinite() ||
            !std::isfinite(sample.yaw_rate) || !sample.variance.allFinite() ||
            (sample.variance.array() < 0.0).any()) return false;
        if (!samples_.empty() && sample.stamp < samples_.back().stamp) {
            if (sample.stamp < samples_.back().stamp - 1.0) samples_.clear();
            else return false;
        }
        samples_.push_back(sample);
        while (samples_.size() > 200 || samples_.back().stamp - samples_.front().stamp > 5.0)
            samples_.pop_front();
        return true;
    }

    bool at(double stamp, double max_age, WheelSample &result) const {
        if (samples_.empty()) return false;
        auto next = std::lower_bound(samples_.begin(), samples_.end(), stamp,
            [](const WheelSample &s, double t) { return s.stamp < t; });
        if (next != samples_.begin() && next != samples_.end()) {
            const auto &prev = *std::prev(next);
            if (stamp - prev.stamp <= max_age && next->stamp - stamp <= max_age) {
                const double a = (stamp - prev.stamp) / (next->stamp - prev.stamp);
                result.stamp = stamp;
                result.velocity = (1.0-a)*prev.velocity + a*next->velocity;
                result.yaw_rate = (1.0-a)*prev.yaw_rate + a*next->yaw_rate;
                result.variance = (1.0-a)*prev.variance + a*next->variance;
                return true;
            }
        }
        auto best = next;
        if (next == samples_.end() || (next != samples_.begin() &&
            stamp - std::prev(next)->stamp < next->stamp - stamp)) best = std::prev(next);
        if (std::abs(best->stamp - stamp) > max_age) return false;
        result = *best;
        return true;
    }

private:
    std::deque<WheelSample> samples_;
};

struct WheelConfig {
    Eigen::Matrix3d imu_from_base = Eigen::Matrix3d::Identity();
    Eigen::Vector3d imu_to_base = Eigen::Vector3d::Zero();
    double velocity_std = 0.05;
    double yaw_rate_std = 0.05;
};

inline Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return m;
}

struct WheelObservation {
    Eigen::Matrix<double, 4, 23> jacobian;
    Eigen::Vector4d residual;
};

// FAST-LIO error-state order: position, rotation, lidar rotation/translation,
// world velocity (12:15), gyro bias (15:18), accel bias, gravity.
inline WheelObservation wheel_observation(
    const Eigen::Matrix3d &world_from_imu, const Eigen::Vector3d &world_velocity,
    const Eigen::Vector3d &gyro_bias, const Eigen::Vector3d &gyro,
    const WheelSample &sample, const WheelConfig &config, double reference_variance) {
    WheelObservation out;
    out.jacobian.setZero();
    const Eigen::Matrix3d base_from_imu = config.imu_from_base.transpose();
    const Eigen::Vector3d vi = world_from_imu.transpose() * world_velocity;
    const Eigen::Vector3d omega = gyro - gyro_bias;
    const Eigen::Vector3d predicted = base_from_imu * (vi + omega.cross(config.imu_to_base));
    out.residual.head<3>() = sample.velocity - predicted;
    out.residual(3) = sample.yaw_rate - (base_from_imu * omega).z();
    out.jacobian.block<3, 3>(0, 3) = base_from_imu * skew(vi);
    out.jacobian.block<3, 3>(0, 12) = base_from_imu * world_from_imu.transpose();
    out.jacobian.block<3, 3>(0, 15) = base_from_imu * skew(config.imu_to_base);
    out.jacobian.block<1, 3>(3, 15) = -base_from_imu.row(2);
    for (int i = 0; i < 4; ++i) {
        const double sigma = i == 3 ? config.yaw_rate_std : config.velocity_std;
        const double scale = std::sqrt(reference_variance / std::max(sigma*sigma, sample.variance(i)));
        out.jacobian.row(i) *= scale;
        out.residual(i) *= scale;
    }
    return out;
}

}  // namespace fast_lio
#endif
