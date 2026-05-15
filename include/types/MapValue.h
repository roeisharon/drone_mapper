#pragma once

namespace dm {

    enum class MapValue : int {
    Empty        =  0,  // position is known to be free
    Occupied     =  1,  // position is known to have an element (wall/floor/ceiling/obstacle)
    NotMapped    = -1,  // position was not reached / could not be mapped
    NotReachable = -2   // position is outside the required mapping boundaries
};

} // namespace dm