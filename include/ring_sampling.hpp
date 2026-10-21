#pragma once
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace fast_lio {
// Interleaved Velodyne packets contain one point per ring before the next
// azimuth. A global index stride can discard entire rings (16 rings / stride 4
// leaves only rings 0,4,8,12). Decimate independently along each ring instead.
class RingSampler {
public:
    RingSampler(int rings, int stride) : stride_(stride) {
        if (rings < 1 || stride < 1) throw std::invalid_argument("Ring count and sampling stride must be positive");
        counts_.resize(rings);
    }
    bool keep(int ring) {
        if (ring < 0 || static_cast<std::size_t>(ring) >= counts_.size()) return false;
        return counts_[ring]++ % stride_ == 0;
    }
private:
    std::size_t stride_;
    std::vector<std::size_t> counts_;
};
}  // namespace fast_lio
