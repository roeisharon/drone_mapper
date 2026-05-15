#pragma once
#include <unordered_map>
#include <vector>
#include <utility>
#include "interfaces/IMapBuilder.h"
#include "IO/ConfigParser.h"
#include "IO/MapIO.h"

namespace dm {

// MapBuilder is the drone's own mapping output store.
// Constructed from MissionConfig so it owns the boundary polygon,
// height limits, and resolution directly from the mission parameters.
//
// The drone calls Set() to record what it discovers and Get() to query it.
// Values default to NotMapped until explicitly set.
// IsInBounds() uses a ray-casting point-in-polygon test for accurate
// boundary checking against any convex or concave boundary shape.
class MapBuilder final : public IMapBuilder {
public:
    // Construct from the mission configuration.
    // Derives all boundary and resolution data from MissionConfig.
    explicit MapBuilder(const MissionConfig& mission);

    // IMapBuilder interface
    [[nodiscard]] MapValue Get(XLength x, YLength y, ZLength z) const override;
    void                   Set(XLength x, YLength y, ZLength z, MapValue value) override;

    // Boundary accessors — expose the mission polygon and height range
    // so the mapping algorithm can query them without holding a config reference.
    [[nodiscard]] const std::vector<std::pair<double,double>>& polygon()   const { return m_polygon; }
    [[nodiscard]] ZLength                                       minHeight() const { return m_minHeight; }
    [[nodiscard]] ZLength                                       maxHeight() const { return m_maxHeight; }

    // Return all cells that were explicitly recorded (used for output and scoring).
    [[nodiscard]] std::vector<MapCell> GetAllCells() const;

private:
    struct Key {
        int ix, iy, iz;
        bool operator==(const Key& o) const noexcept {
            return ix == o.ix && iy == o.iy && iz == o.iz;
        }
    };

    // Hash using multiplicative seed mixing for good distribution across
    // spatially correlated 3D grid indices.
    struct KeyHash {
        std::size_t operator()(const Key& k) const noexcept {
            std::size_t seed = static_cast<std::size_t>(k.ix);
            seed ^= static_cast<std::size_t>(k.iy) + 0x9e3779b9u + (seed << 6) + (seed >> 2);
            seed ^= static_cast<std::size_t>(k.iz) + 0x9e3779b9u + (seed << 6) + (seed >> 2);
            return seed;
        }
    };

    // Snap world coordinates (cm) to the storage resolution grid
    [[nodiscard]] Key  MakeKey(double xCm, double yCm, double zCm) const;

    // Returns false if the position is outside the polygon or height range
    [[nodiscard]] bool IsInBounds(XLength x, YLength y, ZLength z) const;

    // Ray-casting point-in-polygon test (works for any convex or concave polygon)
    [[nodiscard]] bool IsInsidePolygon(double xCm, double yCm) const;

    // m_mission must be declared first — m_polygon holds a reference into it,
    // and C++ initialises members in declaration order.
    const MissionConfig m_mission;

    std::unordered_map<Key, MapValue, KeyHash> m_cells;

    const std::vector<std::pair<double,double>>& m_polygon;   // refers into m_mission (stable lifetime)
    ZLength m_minHeight;
    ZLength m_maxHeight;
    double  m_xyCellCm;   // grid cell size in XY (cm), derived from outputResXY
    double  m_hCellCm;    // grid cell size in Z  (cm), derived from outputResH
};

} // namespace dm