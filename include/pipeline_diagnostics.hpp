#pragma once
// Observer only: sensor stamps and estimator scheduling are never modified.
#include <chrono>
#include <deque>
#include <map>
#include <sstream>
#include <string>
#include <algorithm>

namespace fast_lio {
class PipelineDiagnostics {
 public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  static double ms(Time a, Time b) { return std::chrono::duration<double, std::milli>(a-b).count(); }
  struct Metric {
    unsigned long count = 0;
    double sum = 0, maximum = 0, last = 0;
    void add(double value) { ++count; sum += value; maximum = std::max(maximum, value); last = value; }
  };
  bool enabled = false;
  std::map<std::string, unsigned long> counters;
  std::map<std::string, Metric> timings;
  struct Scan { const void* id; Time receipt, enqueued; };
  std::deque<Scan> queue;
  Scan active{nullptr, {}, {}};
  Time started{}, last_imu_received{};
  bool associated = false;
  double scan_end = 0, last_imu_stamp = 0;
  size_t queue_max = 0;

  void count(const std::string& key) { if (enabled) ++counters[key]; }
  void measure(const std::string& key, double value) { if (enabled) timings[key].add(value); }
  void enqueue(const void* id, Time receipt) {
    if (!enabled) return;
    const auto now = Clock::now();
    queue.push_back({id, receipt, now});
    queue_max = std::max(queue_max, queue.size());
    count("lidar_enqueued"); measure("preprocess_ms", ms(now, receipt));
  }
  void reset_queue() { if (enabled) { count("lidar_clock_reset"); queue.clear(); } }
  void imu(double stamp) {
    if (!enabled) return;
    const auto now = Clock::now();
    if (last_imu_received != Time{}) measure("imu_receipt_interval_ms", ms(now, last_imu_received));
    if (last_imu_stamp != 0) measure("imu_stamp_interval_ms", (stamp-last_imu_stamp)*1000);
    if (stamp < last_imu_stamp) count("imu_clock_reset");
    last_imu_stamp = stamp; last_imu_received = now; count("imu_received");
  }
  void begin(const void* id, double end, size_t imu_count) {
    if (!enabled) return;
    started = Clock::now(); scan_end = end; associated = false;
    // Match the exact queued cloud; never count an unassociated scan as zero latency.
    while (!queue.empty()) {
      const auto candidate = queue.front(); queue.pop_front();
      if (candidate.id == id) { active = candidate; associated = true; break; }
      count("queue_association_mismatch");
    }
    if (associated) measure("queue_wait_ms", ms(started, active.enqueued));
    else count("input_to_output_unassociated");
    count("scans_dequeued"); measure("imu_samples_per_scan", imu_count);
    measure("imu_ahead_of_scan_end_ms", (last_imu_stamp-end)*1000);
  }
  void odometry() {
    if (!enabled) return;
    count("odometry_published");
    if (associated) measure("input_to_odometry_ms", ms(Clock::now(), active.receipt));
  }
  void finish(const std::string& outcome) {
    if (!enabled) return;
    count("outcome_"+outcome); measure("processing_ms", ms(Clock::now(), started));
    if (outcome == "published" && associated)
      measure("input_to_cloud_ms", ms(Clock::now(), active.receipt));
    associated = false;
  }
  std::string json(double ros_now) const {
    const auto now = Clock::now();
    std::ostringstream out;
    out << "{\"schema\":1,\"stamp\":" << std::fixed << ros_now
        << ",\"scan_end\":" << scan_end << ",\"queue_current\":" << queue.size()
        << ",\"queue_max\":" << queue_max << ",\"queue_oldest_age_ms\":"
        << (queue.empty() ? 0 : ms(now, queue.front().receipt))
        << ",\"imu_silence_ms\":" << (last_imu_received == Time{} ? -1 : ms(now, last_imu_received));
    for (const auto& item : counters) out << ",\"" << item.first << "\":" << item.second;
    out << ",\"metrics\":{";
    bool first = true;
    for (const auto& item : timings) {
      if (!first) out << ',';
      first = false;
      const auto& m = item.second;
      out << '"' << item.first << "\":{\"count\":" << m.count << ",\"mean\":" << m.sum/m.count
          << ",\"max\":" << m.maximum << ",\"last\":" << m.last << '}';
    }
    out << "}}";
    return out.str();
  }
  struct Scope {
    PipelineDiagnostics& data;
    std::string outcome = "skipped";
    explicit Scope(PipelineDiagnostics& d): data(d) {}
    ~Scope() { data.finish(outcome); }
  };
};
}  // namespace fast_lio
