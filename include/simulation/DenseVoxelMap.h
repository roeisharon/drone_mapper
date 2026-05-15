#pragma once

#include <vector>
#include "interfaces/IMap3D.h"
#include "IO/MapIO.h"

namespace dm {

// Dense 3D occupancy array backed by a flat contiguous vector.
// Replaces CellMap's unordered_set for the hot beam-tracing path.
// Shape: [x_size][y_size][z_size], row-major (same convention as VoxelGrid).
// Resolution: 1 cm per voxel (floor to nearest integer cm).
class DenseVoxelMap final : public IMap3D {
public:
    explicit DenseVoxelMap(const ParsedMap& parsed);

    [[nodiscard]] int get(const Position3D& pos) const override;

private:
    int    m_xMin{}, m_yMin{}, m_zMin{};
    int    m_xSize{}, m_ySize{}, m_zSize{};
    std::vector<uint8_t> m_grid{};

    [[nodiscard]] bool inBounds(int x, int y, int z) const noexcept;
    [[nodiscard]] std::size_t flatIndex(int x, int y, int z) const noexcept;
};

} // namespace dm
