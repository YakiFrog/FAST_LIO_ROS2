#include "pipeline_diagnostics.hpp"
#include <iostream>
#include <stdexcept>
void check(bool b) { if (!b) throw std::runtime_error("diagnostics assertion"); }
int main() {
  fast_lio::PipelineDiagnostics d;
  int first, second;
  d.enqueue(&first, fast_lio::PipelineDiagnostics::Clock::now());
  check(d.queue.empty());
  d.enabled = true;
  d.enqueue(&first, fast_lio::PipelineDiagnostics::Clock::now());
  d.enqueue(&second, fast_lio::PipelineDiagnostics::Clock::now());
  d.imu(1.0); d.imu(1.005);
  d.begin(&first, 1.004, 10);
  d.odometry(); d.finish("published");
  check(d.queue.size()==1);
  check(d.timings["input_to_cloud_ms"].count==1);
  check(d.counters["odometry_published"]==1);
  d.begin(&first, 2.0, 0); // mismatched cloud cannot invent a zero-latency sample
  d.odometry(); d.finish("empty_imu");
  check(d.timings["input_to_cloud_ms"].count==1);
  check(d.timings["input_to_odometry_ms"].count==1);
  check(d.counters["queue_association_mismatch"]==1);
  check(d.json(2.0).find("\"queue_current\":0")!=std::string::npos);
  std::cout << d.json(2.0) << std::endl;
}
