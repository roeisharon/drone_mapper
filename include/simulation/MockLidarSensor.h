#pragma once
#include "interfaces/IMap3D.h"
#include "interfaces/ILidarSensor.h"
#include "interfaces/IPositionSensor.h"

namespace dm {

// Test/demonstration LiDAR that ray marches through an IMap3D from the current
// IPositionSensor pose.
class MockLidarSensor final : public ILidarSensor {
public:
    MockLidarSensor(LidarConfig config, const IMap3D& map, const IPositionSensor& pos_sensor);

    // scan is relative to current_pos - while the mocklidarsensor knows the drone's real orientation,
    // the drone should not know that. thus the drone will assume that the lidar scans relative to its current orientation
    // The scan orientation is relative to the sensor heading. Returned hit
    // angles are also relative; the mock uses the position sensor heading only
    // internally to trace through the world map.
    [[nodiscard]] ScanResults scan(Orientation rel_scan_orientation) const override;

    [[nodiscard]] const LidarConfig& config() const noexcept;

private:
    // Returns the distance to the first obstacle, or beam_length_max if nothing was hit.
    [[nodiscard]] PhysicalLength traceBeam(const Orientation& beam) const;

    LidarConfig config_;
    const IMap3D& map_;
    const IPositionSensor& pos_sensor_;
};

} // namespace dm
