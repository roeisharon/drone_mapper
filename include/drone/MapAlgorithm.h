#pragma once
#include <queue>
#include <set>
#include <vector>
#include <utility>
#include <cmath>
#include "types/Units.h"
#include "types/MapValue.h"
#include "IO/ConfigParser.h"

namespace dm {

class Drone;

struct GridCell3D {
    int x, y, z;

    bool operator==(const GridCell3D& other) const noexcept {
        return x == other.x && y == other.y && z == other.z;
    }
    bool operator<(const GridCell3D& other) const noexcept {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
    }
};

class MappingAlgorithm {
public:
    explicit MappingAlgorithm(Drone&               drone,
                               const DroneConfig*   config  = nullptr,
                               const MissionConfig* mission = nullptr);

    void Run();

private:
    Drone&               m_drone;
    const DroneConfig*   m_config;
    const MissionConfig* m_mission;

    std::set<GridCell3D>   m_visited;
    std::queue<GridCell3D> m_frontier;

    int    m_minHeight{0};
    int    m_maxHeight{300};
    int    m_currentHeightLevel{150};
    double m_resolution{1.0};
    double m_outputResolution{5.0}; // output grid cell size in cm (matches map input resolution)
    int    m_bfsStep{100};
    double m_currentHeading{0.0};

    // Derived from lidar config — set in constructor
    double m_lidarBeamMin{5.0};
    double m_lidarBeamMax{120.0};
    double m_elevStep{30.0};
    int    m_numAltSteps{5};

    void    ScanAndUpdate();
    void    ScanSingleDirection(double altitudeDeg = 0.0);
    void    ExploreAtCurrentHeight();
    bool    ElevateToNextHeight();

    GridCell3D              WorldToGrid(XLength x, YLength y, ZLength z) const;
    Position3D              GridToWorld(const GridCell3D& cell) const;
    bool                    IsWalkable(const GridCell3D& cell) const;
    bool                    HasClearance(const GridCell3D& to) const;
    std::vector<GridCell3D> FindPath(const GridCell3D& target);
    bool                    MoveToCell(const GridCell3D& target);
    bool                    RotateToFace(XLength targetX, YLength targetY);
    bool                    AdjustHeight(PhysicalLength targetHeight);
};

} // namespace dm