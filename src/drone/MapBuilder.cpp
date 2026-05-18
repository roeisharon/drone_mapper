#include "drone/MapBuilder.h"
#include <cmath>

namespace dm {

MapBuilder::MapBuilder(const MissionConfig& mission)
    : m_mission(mission)
    , m_polygon(m_mission.boundaryPolygon)
    , m_minHeight(mission.minHeight)
    , m_maxHeight(mission.maxHeight)
    , m_xyCellCm(mission.mapResolutionCm)
    , m_hCellCm (mission.mapResolutionCm)
{}

// IMapBuilder - - Get/Set individual cell values
MapValue MapBuilder::Get(XLength x, YLength y, ZLength z) const
{
    if (!IsInBounds(x, y, z)) return MapValue::NotReachable;
    const auto it = m_cells.find(MakeKey(
        x.numerical_value_in(cm),
        y.numerical_value_in(cm),
        z.numerical_value_in(cm)));
    return (it != m_cells.end()) ? it->second : MapValue::NotMapped;
}

void MapBuilder::Set(XLength x, YLength y, ZLength z, MapValue value)
{
    // Snap to the output grid first, then validate bounds on the snapped position.
    // Raw beam-hit coordinates overshoot boundary walls by up to one beam step
    // (e.g. x=200.04 for the x=200 wall), so checking the raw value would
    // silently discard valid boundary-cell hits.
    const double xCm = x.numerical_value_in(cm);
    const double yCm = y.numerical_value_in(cm);
    const double zCm = z.numerical_value_in(cm);
    const Key key = MakeKey(xCm, yCm, zCm);
    const double xSnap = key.ix * m_xyCellCm;
    const double ySnap = key.iy * m_xyCellCm;
    const double zSnap = key.iz * m_hCellCm;

    // Z range check (inclusive at both endpoints).
    const double minZ = m_minHeight.numerical_value_in(cm);
    const double maxZ = m_maxHeight.numerical_value_in(cm);
    if (zSnap < minZ || zSnap > maxZ) return;

    // XY polygon check: test four corners around the snapped point so that
    // cells exactly on the boundary edge (e.g. x=200 in a 0-200 polygon) are
    // accepted even though the polygon ray-cast uses a strict-less-than test.
    if (!m_polygon.empty()) {
        constexpr double eps = 0.1;
        const bool inside = IsInsidePolygon(xSnap - eps, ySnap - eps)
                         || IsInsidePolygon(xSnap + eps, ySnap - eps)
                         || IsInsidePolygon(xSnap - eps, ySnap + eps)
                         || IsInsidePolygon(xSnap + eps, ySnap + eps);
        if (!inside) return;
    }

    auto [it, inserted] = m_cells.emplace(key, value);
    if (!inserted) {
        // Occupied evidence is definitive (comes from a real beam hit).
        // Never downgrade an Occupied cell to Empty — diagonal miss beams
        // can have their empty-path cells snap onto nearby wall positions.
        if (it->second != MapValue::Occupied)
            it->second = value;
    }
}

// GetAllCells — collect every recorded cell for file output and scoring
std::vector<MapCell> MapBuilder::GetAllCells() const
{
    std::vector<MapCell> out;
    out.reserve(m_cells.size());
    for (const auto& [key, val] : m_cells) {
        MapCell cell;
        // Reconstruct world coordinates from grid indices
        cell.x     = (key.ix * m_xyCellCm) * cm;
        cell.y     = (key.iy * m_xyCellCm) * cm;
        cell.z     = (key.iz * m_hCellCm ) * cm;
        cell.value = val;
        out.push_back(cell);
    }
    return out;
}

// Private helpers
MapBuilder::Key MapBuilder::MakeKey(double xCm, double yCm, double zCm) const
{
    return {
        static_cast<int>(std::floor(xCm / m_xyCellCm)),
        static_cast<int>(std::floor(yCm / m_xyCellCm)),
        static_cast<int>(std::floor(zCm / m_hCellCm))
    };
}

bool MapBuilder::IsInBounds(XLength x, YLength y, ZLength z) const
{
    const double zCm = z.numerical_value_in(cm);
    if (zCm < m_minHeight.numerical_value_in(cm)) return false;
    if (zCm > m_maxHeight.numerical_value_in(cm)) return false;
    if (m_polygon.empty()) return true;   // no polygon = no XY restriction
    return IsInsidePolygon(x.numerical_value_in(cm), y.numerical_value_in(cm));
}

// Ray-casting algorithm: count how many polygon edges a horizontal ray from
// (xCm, yCm) crosses. Odd count = inside, even = outside.
bool MapBuilder::IsInsidePolygon(double xCm, double yCm) const
{
    const std::size_t n = m_polygon.size();
    if (n < 3) return false;

    bool inside = false;
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const double xi = m_polygon[i].first,  yi = m_polygon[i].second;
        const double xj = m_polygon[j].first,  yj = m_polygon[j].second;

        const bool intersects =
            ((yi > yCm) != (yj > yCm)) &&
            (xCm < (xj - xi) * (yCm - yi) / (yj - yi) + xi);

        if (intersects) inside = !inside;
    }
    return inside;
}

} // namespace dm