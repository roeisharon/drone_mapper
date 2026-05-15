#pragma once

#include <unordered_set>
#include "interfaces/IMap3D.h"
#include "IO/MapIO.h"

namespace dm {

// CellMap stores the ground-truth building layout loaded from map_input.txt.
// It is given to the mock sensors; the drone algorithm never sees it directly.
// Resolution is 1 cm per voxel (floor to nearest integer cm).
class CellMap final : public IMap3D {
public:
    // Construct from a parsed map (cells with value==Occupied mark solid voxels)
    explicit CellMap(const ParsedMap& parsed);

    // IMap3D: returns 1 if occupied, 0 otherwise.
    [[nodiscard]] int get(const Position3D& pos) const override;

private:
    struct Key { //Make grid coordinates hashable for unordered set
        int x, y, z;
        bool operator==(const Key& o) const noexcept {
            return x == o.x && y == o.y && z == o.z;
        }
    };

    struct KeyHash { // In order to use Key in unordered set
        std::size_t operator()(const Key& k) const noexcept {
            std::size_t h = 2166136261u;
            h ^= static_cast<std::size_t>(k.x + 100000); h *= 16777619u;
            h ^= static_cast<std::size_t>(k.y + 100000); h *= 16777619u;
            h ^= static_cast<std::size_t>(k.z + 100000); h *= 16777619u;
            return h;
        }
    };

    static Key makeKey(double x, double y, double z);

    std::unordered_set<Key, KeyHash> m_occupied; // Set of occupied voxels (ground-truth map)
};

} // namespace dm