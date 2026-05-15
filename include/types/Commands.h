#pragma once
#include <variant>
#include "types/Units.h"

namespace dm {

struct Rotate  { HorizontalAngle angle; };
struct Advance { PhysicalLength   distance; };
struct Elevate { PhysicalLength   distance; };
struct Scan   { Orientation      scan_orientation{}; };
struct GetLocation {};
struct Finish    {};

using DroneCommands = std::variant<
    Rotate,
    Advance,
    Elevate,
    Scan,
    GetLocation,
    Finish
>;

} // namespace dm