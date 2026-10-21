#include "lidar_quality.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
void check(bool b) { if (!b) throw std::runtime_error("lidar quality assertion"); }
int main() {
  using Guard = fast_lio::LidarQualityGuard;
  Guard guard;
  fast_lio::LidarQualityStats good;
  good.input_points=1000; good.matched_points=700; good.mean_residual=.025;
  good.normal_information=Eigen::Vector3d(200,200,300).asDiagonal();
  guard.begin_scan(); guard.evaluate({}); check(guard.weight==1 && guard.allow_map_update());
  guard.config.enabled=true; guard.config.recovery_scans=3; guard.config.validate();
  guard.begin_scan(); guard.evaluate(good); check(guard.mode==Guard::Mode::Good);
  auto weak=good; weak.normal_information=Eigen::Vector3d(0,0,700).asDiagonal();
  guard.begin_scan(); guard.evaluate(weak);
  check(guard.mode==Guard::Mode::Degraded && guard.weight==.2 && !guard.allow_map_update());
  // An IEKF rematch in the same scan must not advance recovery or change weight.
  guard.evaluate(good); check(guard.mode==Guard::Mode::Degraded && guard.good_streak==0);
  for(int i=1;i<=3;++i) {
    guard.begin_scan(); guard.evaluate(good);
    check(guard.good_streak==i);
    check(guard.allow_map_update()==(i==3));
  }
  auto bad=good; bad.matched_points=0;
  guard.begin_scan(); guard.evaluate(bad);
  check(guard.weight==0 && guard.mode==Guard::Mode::Rejected && !guard.allow_map_update());
  guard.finish_scan(); check(guard.rejected_scans==1 && guard.frozen_scans==1);
  guard.begin_scan(); guard.evaluate(good); check(guard.mode==Guard::Mode::Recovering);
  bad=good; bad.mean_residual=.5;
  guard.begin_scan(); guard.evaluate(bad); check(guard.reason=="high_residual" && guard.good_streak==0);
  bad=good; bad.input_points=10000;
  guard.begin_scan(); guard.evaluate(bad); check(guard.reason=="low_match_ratio");
  bad=good; bad.normal_information(0,0)=std::numeric_limits<double>::quiet_NaN();
  guard.begin_scan(); guard.evaluate(bad); check(guard.reason=="nonfinite_measurement");
  guard.begin_scan(); guard.evaluate(good);
  check(!guard.correction_valid(.6,.01) && guard.reason=="excessive_correction");
  check(!guard.allow_map_update() && guard.weight==0);
  guard.begin_scan(); guard.evaluate(good);
  check(!guard.correction_valid(.01,.2));
  guard.begin_scan(); guard.evaluate(good);
  check(!guard.correction_valid(std::numeric_limits<double>::infinity(),0));
  const auto json=guard.json(false); check(json.find("inf")==std::string::npos);
  check(json.find("\"map_update_allowed\":false")!=std::string::npos);
  guard.config.enabled=false; guard.begin_scan(); guard.evaluate(bad);
  check(guard.weight==1 && guard.allow_map_update());
  guard.config.degraded_weight=0;
  bool threw=false; try {guard.config.validate();}catch(const std::invalid_argument&) {threw=true;}
  check(threw);
  std::cout << "lidar quality: rejection, soft weighting, recovery, invalid data and disabled compatibility passed\n";
}
