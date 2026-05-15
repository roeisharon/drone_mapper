#pragma once
#include <cstddef>
#include <vector>
#include "types/Units.h"

namespace dm {

// Static scan settings for a LiDAR-like sensor.
struct LidarConfig {
    PhysicalLength beam_length_min{};
    PhysicalLength beam_length_max{};
    PhysicalLength circle_spacing{};
    std::size_t fov_circles{};
};

// A single LiDAR return. The angle is relative to the requested scan orientation.
struct LidarHit {
    PhysicalLength distance{};
    Orientation angle{};
};

using ScanResults = std::vector<LidarHit>;

class ILidarSensor {
public:
    virtual ~ILidarSensor() = default;

    // Scans around the requested relative orientation and returns all beam hits.
    [[nodiscard]] virtual ScanResults scan(Orientation scan_orientation) const = 0;
};

} // namespace dm
