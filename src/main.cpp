#include <iostream>
#include <filesystem>
#include <memory>
#include <string>
#include <algorithm>
#include <set>
#include <cmath>

#include "IO/ConfigParser.h"
#include "IO/MapIO.h"
#include "IO/Score.h"
#include "IO/ErrorLogger.h"

#include "simulation/CellMap.h"
#include "simulation/DenseVoxelMap.h"
#include "simulation/SimulationState.h"
#include "simulation/MockLidarSensor.h"
#include "simulation/MockPositionSensor.h"
#include "simulation/MockMovementDriver.h"

#include "drone/Drone.h"
#include "drone/MapBuilder.h"
#include "drone/MapAlgorithm.h"

int main(int argc, char* argv[])
{
    // ── 1. Determine working directory ───────────────────────────────────────
    std::filesystem::path ioPath =
        (argc >= 2) ? std::filesystem::path(argv[1])
                    : std::filesystem::current_path();

    if (!std::filesystem::exists(ioPath)) {
        std::cerr << "Error: path does not exist: " << ioPath << "\n";
        return 1;
    }

    // ── 2. Load configuration files ──────────────────────────────────────────
    dm::ErrorLogger logger;

    dm::DroneConfig   droneCfg;
    dm::MissionConfig missionCfg;

    const bool droneOk   = dm::ParseDroneConfig  (ioPath / "drone_config.txt",   droneCfg,   logger);
    const bool missionOk = dm::ParseMissionConfig (ioPath / "mission_config.txt", missionCfg, logger);

    if (!droneOk || !missionOk) {
        std::cerr << "Unrecoverable error: failed to open required config file(s).\n";
        logger.FlushToFile(ioPath);
        return 1;
    }

    // ── 3. Load ground-truth map ──────────────────────────────────────────────
    const dm::ParsedMap groundTruthMap =
        dm::ParseMapFile(ioPath / "map_input.txt", logger);

    if (!groundTruthMap.valid) {
        std::cerr << "Unrecoverable error: failed to parse map_input.txt.\n";
        logger.FlushToFile(ioPath);
        return 1;
    }

    // ── 4. Validate that the requested map resolution matches the input map ──
    // Assignment note: "You may assume a single supported resolution. Any
    // requested resolution that is not the one supported may yield an error."
    // We detect the actual grid step from the input map and compare against
    // map_resolution_cm from the mission config.
    {
        using dm::cm;
        // Collect all unique coordinate values on each axis and find the
        // minimum non-zero difference — that is the map's native resolution.
        std::set<int> xs, ys, zs;
        for (const auto& cell : groundTruthMap.cells) {
            xs.insert(static_cast<int>(std::round(cell.x.numerical_value_in(cm))));
            ys.insert(static_cast<int>(std::round(cell.y.numerical_value_in(cm))));
            zs.insert(static_cast<int>(std::round(cell.z.numerical_value_in(cm))));
        }

        auto minStep = [](const std::set<int>& vals) -> int {
            int step = 0;
            auto it = vals.begin();
            if (it == vals.end()) return 0;
            auto prev = it++;
            while (it != vals.end()) {
                const int d = *it - *prev;
                if (d > 0 && (step == 0 || d < step)) step = d;
                prev = it++;
            }
            return step;
        };

        const int stepX = minStep(xs);
        const int stepY = minStep(ys);
        const int stepZ = minStep(zs);
        // Use the minimum non-zero step across all axes as the detected resolution.
        int detectedRes = 0;
        for (int s : {stepX, stepY, stepZ})
            if (s > 0 && (detectedRes == 0 || s < detectedRes)) detectedRes = s;

        if (detectedRes > 0) {
            const int requestedRes = static_cast<int>(std::round(missionCfg.mapResolutionCm));
            // Config map resolution must match the detected resolution from the input map, otherwise the mapping will be inaccurate.
            if (requestedRes != detectedRes) {
                std::cerr << "Error: map_resolution_cm=" << requestedRes
                          << " does not match the input map resolution ("
                          << detectedRes << " cm). Aborting.\n";
                logger.FlushToFile(ioPath);
                return 1;
            }
        }
    }

    // ── 5. Build simulation objects ───────────────────────────────────────────
    auto simState = std::make_shared<dm::SimulationState>();
    simState->position = {
        missionCfg.startX,
        missionCfg.startY,
        missionCfg.startHeight
    };
    simState->orientation = { 0.0 * dm::deg, 0.0 * dm::deg };

    const dm::DenseVoxelMap cellMap(groundTruthMap);

    auto positionSensor = std::make_shared<dm::MockPositionSensor>(
        std::shared_ptr<const dm::SimulationState>(simState));

    const dm::LidarConfig lidarCfg {
        droneCfg.lidarMinBeam,
        droneCfg.lidarMaxBeam,
        droneCfg.lidarCircleSpacing,
        droneCfg.lidarFOVC
    };
    dm::MockLidarSensor   lidarSensor(lidarCfg, cellMap, *positionSensor);
    dm::MockMovementDriver movementDriver(simState, droneCfg, cellMap);

    dm::MapBuilder mapBuilder(missionCfg);

    // ── 6. Create and run the drone ───────────────────────────────────────────
    dm::Drone drone(lidarSensor, *positionSensor, movementDriver, mapBuilder);
    dm::MappingAlgorithm algorithm(drone, &droneCfg, &missionCfg);

    std::cout << "Starting mapping mission...\n";
    algorithm.Run();
    std::cout << "Mapping complete.\n";

    // ── 7. Write output map ───────────────────────────────────────────────────
    const std::vector<dm::MapCell> outputCells = mapBuilder.GetAllCells();
    const std::filesystem::path outputPath = ioPath / "map_output.txt";

    if (!dm::WriteMapFile(outputPath, groundTruthMap.bounds, outputCells)) {
        std::cerr << "Warning: could not write map_output.txt\n";
    } else {
        std::cout << "Output map written to: " << outputPath << "\n";
    }

    // ── 8. Score ──────────────────────────────────────────────────────────────
    const double score = dm::Score(outputCells, groundTruthMap.cells);
    std::cout << "Mapping score: " << score << " / 100\n";

    // ── 9. Flush any input errors ─────────────────────────────────────────────
    logger.FlushToFile(ioPath);
    if (logger.HasErrors()) {
        std::cout << "Input errors were logged to: " << (ioPath / "input_errors.txt") << "\n";
    }

    return 0;
}