#pragma once
#include <vector>
#include "IO/MapIO.h"

namespace dm {

// Will compute score based on true/false positives/negatives in the mapped cells compared to the ground truth cells.
double Score(const std::vector<MapCell>& mapped,
                      const std::vector<MapCell>& groundTruth);

} // namespace dm