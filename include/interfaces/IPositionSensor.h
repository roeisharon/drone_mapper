#pragma once
#include "types/Units.h"

namespace dm {

// Supplies the current world-space pose of the sensor platform.
class IPositionSensor {
public:
    virtual ~IPositionSensor() = default;

    [[nodiscard]] virtual Position3D position() const = 0;
    [[nodiscard]] virtual Orientation heading() const = 0;
};

} // namespace dm
