#pragma once
#include "types/Units.h"
#include "types/MapValue.h"

namespace dm {

class IMapBuilder {
public:
    virtual ~IMapBuilder() = default;

    virtual MapValue Get(XLength x, YLength y, ZLength z) const = 0; // Returns MapValue::Unknown if the position is out of bounds or has not been set yet.
    virtual void     Set(XLength x, YLength y, ZLength z, MapValue value) = 0; // Sets the value at the given position. Does nothing if the position is out of bounds.
};

} // namespace dm