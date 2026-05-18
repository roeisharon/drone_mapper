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
//   The output map is expected to contain the augmented full output, which
//   includes -1 (NotMapped) and -2 (NotReachable) values in addition to the
//   drone's scanned 0 (Empty) and 1 (Occupied) cells.
//
//   For every GT cell with value Occupied or Empty, look up the drone's output:
//
//     Output = correct value (0/1 matching GT) → +1 correct,  counts in total
//     Output = wrong value   (0/1 not matching) → -0.5 penalty, counts in total
//     Output = -1 (NotMapped, inside mission)   → -0.5 penalty, counts in total
//     Output = -2 (NotReachable, outside mission)→ excluded entirely from total
//     Output = missing from output map           → -0.5 penalty, counts in total
//
//   score = max(0, correct - penalty) / total * 100

double Score(const std::vector<MapCell>& mapped,
             const std::vector<MapCell>& groundTruth)
{
    if (groundTruth.empty()) return 0.0;

    // Build a lookup table from the full augmented output (includes -1/-2).
    std::unordered_map<Key, MapValue, KeyHash> mappedMap;
    mappedMap.reserve(mapped.size());
    for (const auto& cell : mapped) {
        mappedMap[makeKey(cell)] = cell.value;
    }

    double correct  = 0.0;
    double penalty  = 0.0;
    int    total    = 0;

    for (const auto& gt : groundTruth) {
        // Only evaluate cells that are definitively Occupied or Empty in the GT.
        if (gt.value != MapValue::Occupied && gt.value != MapValue::Empty) continue;

        auto it = mappedMap.find(makeKey(gt));

        // -2 (NotReachable): cell is outside the mission bounds.
        // Exclude it entirely — not in numerator, not in denominator.
        if (it != mappedMap.end() && it->second == MapValue::NotReachable) continue;

        // All other cases count toward the total (denominator).
        ++total;

        if (it == mappedMap.end()) {
            // Not present in output at all — treat same as NotMapped.
            penalty += 0.5;
            continue;
        }

        const MapValue dv = it->second;
        if (dv == gt.value) {
            // Correct classification.
            correct += 1.0;
        } else if (dv == MapValue::Occupied || dv == MapValue::Empty) {
            // Wrong classification (0 vs 1 or vice-versa).
            penalty += 0.5;
        } else if (dv == MapValue::NotMapped) {
            // -1: inside mission bounds but drone failed to map it.
            penalty += 0.5;
        }
    }

    if (total == 0) return 0.0;

    const double numerator = std::max(0.0, correct - penalty);
    return (numerator / static_cast<double>(total)) * 100.0;
}

} // namespace dm
