#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

namespace fast_lio {
struct LidarQualityConfig {
  bool enabled = false;
  int min_points = 30;
  double min_ratio = .10;
  double max_residual = .15;
  double weak_geometry_ratio = .003;
  double degraded_weight = .20;
  int recovery_scans = 10;
  double max_position_correction = .50;
  double max_rotation_correction = .15;
  void validate() const {
    if (min_points < 1 || recovery_scans < 1 ||
        !std::isfinite(min_ratio) || min_ratio <= 0 || min_ratio > 1 ||
        !std::isfinite(max_residual) || max_residual <= 0 ||
        !std::isfinite(weak_geometry_ratio) || weak_geometry_ratio <= 0 || weak_geometry_ratio > 1 ||
        !std::isfinite(degraded_weight) || degraded_weight <= 0 || degraded_weight >= 1 ||
        !std::isfinite(max_position_correction) || max_position_correction <= 0 ||
        !std::isfinite(max_rotation_correction) || max_rotation_correction <= 0)
      throw std::invalid_argument("Invalid lidar_quality parameters");
  }
};

struct LidarQualityStats {
  int input_points = 0, matched_points = 0;
  double mean_residual = 0;
  Eigen::Matrix3d normal_information = Eigen::Matrix3d::Zero();
};

// One decision per scan, frozen across IEKF iterations. Recovery needs consecutive
// good scans; no map growth while weak/rejected/recovering. No pose/frame switching.
class LidarQualityGuard {
 public:
  enum class Mode { Disabled = 0, Good = 1, Degraded = 2, Rejected = 3, Recovering = 4 };
  LidarQualityConfig config;
  Mode mode = Mode::Disabled;
  std::string reason = "disabled";
  double weight = 1, ratio = 0, geometry_ratio = 0, residual = 0;
  double position_correction = 0, rotation_correction = 0;
  int points = 0, good_streak = 0;
  unsigned long rejected_scans = 0, frozen_scans = 0;

  void begin_scan() {
    evaluated_ = false;
    position_correction = rotation_correction = 0;
    if (!config.enabled) { mode = Mode::Disabled; reason = "disabled"; weight = 1; }
  }
  void evaluate(const LidarQualityStats& stats) {
    if (evaluated_) return;
    evaluated_ = true;
    points = stats.matched_points;
    ratio = stats.input_points > 0 ? double(points)/stats.input_points : 0;
    residual = stats.mean_residual;
    geometry_ratio = 0;
    const bool finite = std::isfinite(residual) && stats.normal_information.allFinite();
    if (finite && points > 0) {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(stats.normal_information);
      if (solver.info() == Eigen::Success && solver.eigenvalues().maxCoeff() > 0)
        geometry_ratio = std::max(0., solver.eigenvalues().minCoeff()) / solver.eigenvalues().maxCoeff();
    }
    if (!config.enabled) return;
    if (!finite) reject("nonfinite_measurement");
    else if (points < config.min_points) reject("too_few_matches");
    else if (ratio < config.min_ratio) reject("low_match_ratio");
    else if (residual > config.max_residual) reject("high_residual");
    else if (geometry_ratio < config.weak_geometry_ratio || ratio < 2*config.min_ratio ||
             residual > .7*config.max_residual) {
      mode = Mode::Degraded; weight = config.degraded_weight;
      reason = geometry_ratio < config.weak_geometry_ratio ? "weak_geometry" : "weak_matching";
      recovery_required_ = true; good_streak = 0;
    } else if (recovery_required_ && ++good_streak < config.recovery_scans) {
      mode = Mode::Recovering; weight = config.degraded_weight; reason = "confirming_recovery";
    } else {
      mode = Mode::Good; weight = 1; reason = "good"; recovery_required_ = false;
    }
  }
  bool correction_valid(double position, double rotation) {
    position_correction = position; rotation_correction = rotation;
    if (!config.enabled) return true;
    if (!std::isfinite(position) || !std::isfinite(rotation)) {
      reject("nonfinite_correction"); return false;
    }
    if (position > config.max_position_correction || rotation > config.max_rotation_correction) {
      reject("excessive_correction"); return false;
    }
    return true;
  }
  void reject(const char* why) {
    mode = Mode::Rejected; reason = why; weight = 0;
    recovery_required_ = true; good_streak = 0;
  }
  bool allow_map_update() const { return !config.enabled || mode == Mode::Good; }
  void finish_scan() {
    if (!config.enabled) return;
    if (mode == Mode::Rejected) ++rejected_scans;
    if (!allow_map_update()) ++frozen_scans;
  }
  std::string json(bool wheel_available) const {
    std::ostringstream s;
    s << "{\"mode\":\"" << mode_name() << "\",\"reason\":\"" << reason
      << "\",\"lidar_weight\":" << weight << ",\"map_update_allowed\":"
      << (allow_map_update() ? "true" : "false") << ",\"wheel_available\":"
      << (wheel_available ? "true" : "false") << ",\"matched_points\":" << points
      << ",\"match_ratio\":" << ratio << ",\"geometry_ratio\":" << geometry_ratio
      << ",\"mean_residual_m\":" << finite_or_zero(residual)
      << ",\"position_correction_m\":" << finite_or_zero(position_correction)
      << ",\"rotation_correction_rad\":" << finite_or_zero(rotation_correction)
      << ",\"recovery_good_scans\":" << good_streak << ",\"rejected_scans\":" << rejected_scans
      << ",\"map_frozen_scans\":" << frozen_scans << '}';
    return s.str();
  }
  const char* mode_name() const {
    switch (mode) {
      case Mode::Good: return "good";
      case Mode::Degraded: return "degraded";
      case Mode::Rejected: return "rejected";
      case Mode::Recovering: return "recovering";
      default: return "disabled";
    }
  }
 private:
  bool evaluated_ = false, recovery_required_ = false;
  static double finite_or_zero(double v) { return std::isfinite(v) ? v : 0.; }
};
}  // namespace fast_lio
