#pragma once

#include <filesystem>
#include <vector>
#include "types/MapValue.h"
#include "types/Units.h"
#include "IO/ErrorLogger.h"

namespace dm {

struct MapBounds {
    PhysicalLength x_min{}, x_max{};
    PhysicalLength y_min{}, y_max{};
    PhysicalLength z_min{}, z_max{};
};

struct MapCell {
    PhysicalLength x{};
    PhysicalLength y{};
    PhysicalLength z{};
    MapValue value{MapValue::NotMapped};
};

struct ParsedMap {
    bool valid{false};
    MapBounds bounds;
    std::vector<MapCell> cells;
};

ParsedMap ParseMapFile(const std::filesystem::path& path,
                       ErrorLogger& logger);

bool WriteMapFile(const std::filesystem::path& path,
                  const MapBounds&              bounds,
                  const std::vector<MapCell>&   cells);

} // namespace dm