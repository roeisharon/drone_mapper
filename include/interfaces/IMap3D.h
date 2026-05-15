#pragma once
#include "types/Units.h"

namespace dm {

// Read-only 3D occupancy map interface used by LiDAR implementations.
class IMap3D {
public:
    virtual ~IMap3D() = default;

    // Returns the voxel value at a physical position. Implementations should
    // return 0 for empty or out-of-bounds space and non-zero for occupied space.
    [[nodiscard]] virtual int get(const Position3D& pos) const = 0;
};

} // namespace dm
