#ifndef FAST_LIO_SCAN_IMU_SYNC_HPP
#define FAST_LIO_SCAN_IMU_SYNC_HPP

namespace fast_lio {

// Include the endpoint: simulated snapshot clouds and IMUs share exactly the
// same stamp. Excluding it leaves every other measurement empty at equal rates.
template<typename Buffer, typename Samples, typename StampOf>
void collect_scan_imu(Buffer &buffer, double scan_end, Samples &samples, StampOf stamp_of) {
    samples.clear();
    while (!buffer.empty() && stamp_of(buffer.front()) <= scan_end) {
        samples.push_back(buffer.front());
        buffer.pop_front();
    }
}

}  // namespace fast_lio
#endif
