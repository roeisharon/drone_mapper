#include "simulation/DenseVoxelMap.h"
#include <cmath>
#include <set>

namespace dm {

DenseVoxelMap::DenseVoxelMap(const ParsedMap& parsed)
{
    m_xMin = static_cast<int>(std::floor(parsed.bounds.x_min.numerical_value_in(cm)));
    m_yMin = static_cast<int>(std::floor(parsed.bounds.y_min.numerical_value_in(cm)));
    m_zMin = static_cast<int>(std::floor(parsed.bounds.z_min.numerical_value_in(cm)));

    const int xMax = static_cast<int>(std::ceil(parsed.bounds.x_max.numerical_value_in(cm)));
    const int yMax = static_cast<int>(std::ceil(parsed.bounds.y_max.numerical_value_in(cm)));
    const int zMax = static_cast<int>(std::ceil(parsed.bounds.z_max.numerical_value_in(cm)));

    m_xSize = xMax - m_xMin + 1;
    m_ySize = yMax - m_yMin + 1;
    m_zSize = zMax - m_zMin + 1;

    m_grid.assign(static_cast<std::size_t>(m_xSize) *
                  static_cast<std::size_t>(m_ySize) *
                  static_cast<std::size_t>(m_zSize), 0u);

    // Compute the map's native grid resolution so each cell can be filled as a
    // solid block. Without this, each 5-cm map cell only marks one 1-cm voxel,
    // leaving 4-cm gaps that diagonal lidar beams can slip through.
    std::set<int> xs, ys, zs;
    for (const auto& cell : parsed.cells) {
        xs.insert(static_cast<int>(std::floor(cell.x.numerical_value_in(cm))));
        ys.insert(static_cast<int>(std::floor(cell.y.numerical_value_in(cm))));
        zs.insert(static_cast<int>(std::floor(cell.z.numerical_value_in(cm))));
    }
    auto minStep = [](const std::set<int>& vals) -> int {
        int step = 1;
        auto it = vals.begin();
        if (it == vals.end()) return 1;
        auto prev = it++;
        while (it != vals.end()) {
            const int d = *it - *prev;
            if (d > 0 && d < step) step = d;
            prev = it++;
        }
        return step;
    };
    const int resX = minStep(xs);
    const int resY = minStep(ys);
    const int resZ = minStep(zs);

    for (const auto& cell : parsed.cells) {
        if (cell.value != MapValue::Occupied) continue;
        const int ox = static_cast<int>(std::floor(cell.x.numerical_value_in(cm))) - m_xMin;
        const int oy = static_cast<int>(std::floor(cell.y.numerical_value_in(cm))) - m_yMin;
        const int oz = static_cast<int>(std::floor(cell.z.numerical_value_in(cm))) - m_zMin;
        for (int dx = 0; dx < resX; ++dx)
            for (int dy = 0; dy < resY; ++dy)
                for (int dz = 0; dz < resZ; ++dz)
                    if (inBounds(ox + dx, oy + dy, oz + dz))
                        m_grid[flatIndex(ox + dx, oy + dy, oz + dz)] = 1u;
    }
}

int DenseVoxelMap::get(const Position3D& pos) const
{
    const int x = static_cast<int>(std::floor(pos.x.numerical_value_in(cm))) - m_xMin;
    const int y = static_cast<int>(std::floor(pos.y.numerical_value_in(cm))) - m_yMin;
    const int z = static_cast<int>(std::floor(pos.z.numerical_value_in(cm))) - m_zMin;
    if (!inBounds(x, y, z)) return 0;
    return static_cast<int>(m_grid[flatIndex(x, y, z)]);
}

bool DenseVoxelMap::inBounds(int x, int y, int z) const noexcept
{
    return x >= 0 && x < m_xSize &&
           y >= 0 && y < m_ySize &&
           z >= 0 && z < m_zSize;
}

std::size_t DenseVoxelMap::flatIndex(int x, int y, int z) const noexcept
{
    return static_cast<std::size_t>(x) * static_cast<std::size_t>(m_ySize) *
                                         static_cast<std::size_t>(m_zSize)
         + static_cast<std::size_t>(y) * static_cast<std::size_t>(m_zSize)
         + static_cast<std::size_t>(z);
}

} // namespace dm
