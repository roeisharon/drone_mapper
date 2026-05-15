#include "simulation/CellMap.h"
#include <cmath>

namespace dm {

CellMap::CellMap(const ParsedMap& parsed)
{
    for (const auto& cell : parsed.cells) {
        if (cell.value == MapValue::Occupied) {
            m_occupied.insert(makeKey(
                cell.x.numerical_value_in(cm),
                cell.y.numerical_value_in(cm),
                cell.z.numerical_value_in(cm)));
        }
    }
}

int CellMap::get(const Position3D& pos) const
{
    const auto key = makeKey(
        pos.x.numerical_value_in(cm),
        pos.y.numerical_value_in(cm),
        pos.z.numerical_value_in(cm));
    return m_occupied.count(key) ? 1 : 0;
}

CellMap::Key CellMap::makeKey(double x, double y, double z)
{
    return {
        static_cast<int>(std::floor(x)),
        static_cast<int>(std::floor(y)),
        static_cast<int>(std::floor(z))
    };
}

} // namespace dm