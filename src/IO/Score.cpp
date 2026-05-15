#include "IO/Score.h"
#include <cmath>
#include <unordered_map>
#include <tuple>

namespace dm {

namespace {

// Snap a position to integer-cm key for cell-by-cell comparison.
struct Key {
    int x, y, z;
    bool operator==(const Key& o) const noexcept {
        return x == o.x && y == o.y && z == o.z;
    }
};
struct KeyHash {
    std::size_t operator()(const Key& k) const noexcept {
        std::size_t h = 2166136261u;
        h ^= static_cast<std::size_t>(k.x + 100000); h *= 16777619u;
        h ^= static_cast<std::size_t>(k.y + 100000); h *= 16777619u;
        h ^= static_cast<std::size_t>(k.z + 100000); h *= 16777619u;
        return h;
    }
};

Key makeKey(const MapCell& c)
{
    return {
        static_cast<int>(std::round(c.x.numerical_value_in(cm))),
        static_cast<int>(std::round(c.y.numerical_value_in(cm))),
        static_cast<int>(std::round(c.z.numerical_value_in(cm)))
    };
}

} // namespace

// Score formula:
//
//   For every cell in the ground-truth map that is Occupied or Empty:
//     - If the drone mapped it with the correct value → +1 (true positive/negative)
//     - If the drone mapped it with the wrong value   → 0
//     - If the drone left it NotMapped                → 0
//
//   score = (correct / total_ground_truth_cells) * 100
//
//   Additionally, incorrectly mapped cells (drone says Occupied but GT says Empty
//   and vice-versa) count as penalties: each subtracts 0.5 from the correct count
//   (clamped to zero).  This discourages the drone from wildly guessing.
double Score(const std::vector<MapCell>& mapped,
             const std::vector<MapCell>& groundTruth)
{
    if (groundTruth.empty()) return 0.0;

    // Build a lookup table from the drone's output
    std::unordered_map<Key, MapValue, KeyHash> mappedMap;
    mappedMap.reserve(mapped.size());
    for (const auto& cell : mapped) {
        mappedMap[makeKey(cell)] = cell.value;
    }

    double correct  = 0.0;
    double penalty  = 0.0;
    int    total    = 0;

    for (const auto& gt : groundTruth) {
        // Only evaluate cells that are definitively Occupied or Empty in the GT
        if (gt.value != MapValue::Occupied && gt.value != MapValue::Empty) continue;

        ++total;

        auto it = mappedMap.find(makeKey(gt));
        if (it == mappedMap.end()) {
            // Not mapped – no credit, no penalty
            continue;
        }

        const MapValue dv = it->second;
        if (dv == gt.value) {
            correct += 1.0;
        } else if (dv == MapValue::Occupied || dv == MapValue::Empty) {
            // Wrong classification
            penalty += 0.5;
        }
        // NotMapped / NotReachable entries get neither credit nor penalty
    }

    if (total == 0) return 0.0;

    const double numerator = std::max(0.0, correct - penalty);
    return (numerator / static_cast<double>(total)) * 100.0;
}

} // namespace dm
