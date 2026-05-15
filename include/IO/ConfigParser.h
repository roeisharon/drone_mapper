#pragma once

#include <filesystem>
#include <vector>
#include <utility>
#include "IO/ErrorLogger.h"
#include "types/Units.h"

namespace dm {
    struct DroneConfig {
        PhysicalLength minPassWidth{}; // minimal width for the drone to pass through
        PhysicalLength minPassLength{}; // minimal length for the drone to pass through 
        PhysicalLength minPassHeight{}; // minimal height for the drone to pass through
        HorizontalAngle maxRotate  {}; // maximum horizontal rotation angle
        PhysicalLength  maxAdvance {}; // maximum distance the drone can advance in one step
        PhysicalLength  maxElevate {}; // maximum distance the drone can elevate in one step
        PhysicalLength  lidarMinBeam {}; // minimum distance for the lidar to detect an obstacle
        PhysicalLength  lidarMaxBeam {}; // maximum distance for the lidar to detect an obstacle
        PhysicalLength  lidarCircleSpacing {}; // distance between two adjacent circles in the lidar scan pattern
        std::size_t lidarFOVC{}; // number of beam circles
    };

    struct MissionConfig {
        std::vector<std::pair<double, double>> boundaryPolygon; // list of points defining the mission boundary (x, y)
        ZLength minHeight {}; // minimum mission height boundary
        ZLength maxHeight {}; // maximum mission height boundary
        XLength startX      {}; // starting position of the drone (x, y, height)
        YLength startY      {};
        ZLength startHeight {};
        double outputResXY {}; // number of decimal places after the dot for the output coordinates (x, y)
        double outputResH  {}; // number of decimal places after the dot for the output height
        double mapResolutionCm{5.0}; // output grid cell size in cm (must match input map resolution)
    };

    bool ParseDroneConfig(const std::filesystem::path& path, DroneConfig& droneConfig, ErrorLogger& logger);
    bool ParseMissionConfig(const std::filesystem::path& path, MissionConfig& missionConfig, ErrorLogger& logger);

} // namespace dm 