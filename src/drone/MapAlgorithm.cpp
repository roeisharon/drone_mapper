#include "drone/MapAlgorithm.h"
#include "drone/Drone.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <map>
#include <functional>
#include <limits>
#include <vector>

namespace dm {

MappingAlgorithm::MappingAlgorithm(Drone&               drone,
                                    const DroneConfig*   config,
                                    const MissionConfig* mission)
    : m_drone(drone)
    , m_config(config)
    , m_mission(mission)
{
    if (mission) {
        m_minHeight = static_cast<int>(
            mission->minHeight.numerical_value_in(cm));
        m_maxHeight = static_cast<int>(
            mission->maxHeight.numerical_value_in(cm));

        Position3D startPos = m_drone.GetLocation();
        m_currentHeightLevel = static_cast<int>(
            startPos.z.numerical_value_in(cm));

        if (mission->mapResolutionCm > 0.0)
            m_outputResolution = mission->mapResolutionCm;
    }

    if (config) {
        m_lidarBeamMin = config->lidarMinBeam.numerical_value_in(cm);
        m_lidarBeamMax = config->lidarMaxBeam.numerical_value_in(cm);

        // BFS step: half the lidar range guarantees overlapping coverage between positions
        m_bfsStep = std::max(10, static_cast<int>(m_lidarBeamMax * 0.5));

        // Elevation step: vertical reach of the outermost lidar circle at max range.
        // This is how far apart two drone heights can be while still having altitude-beam
        // overlap on vertical surfaces. The 0.8 factor ensures a conservative overlap.
        const double outerRadius = static_cast<double>(
            config->lidarFOVC > 1 ? config->lidarFOVC - 1 : 1)
            * config->lidarCircleSpacing.numerical_value_in(cm) / 2.0;
        const double maxAltAngle = std::atan2(outerRadius, m_lidarBeamMin);
        const double vertCoverage = m_lidarBeamMax * std::tan(maxAltAngle);
        m_elevStep = std::max(
            config->maxElevate.numerical_value_in(cm),
            vertCoverage * 0.8);
        // Cap: always visit at least 3 height slices across the mission range
        const double heightRange = static_cast<double>(m_maxHeight - m_minHeight);
        if (heightRange > 0.0)
            m_elevStep = std::min(m_elevStep, heightRange / 2.0);

        // FOVC already sweeps altitude: the outer circle sits at
        // atan2((FOVC-1)*D, beam_min) from horizontal, so explicit altitude
        // tilts are only needed for the remaining uncovered angle.
        const double fovOuterDeg = (config->lidarFOVC > 1)
            ? std::atan2(static_cast<double>(config->lidarFOVC - 1)
                             * config->lidarCircleSpacing.numerical_value_in(cm),
                         m_lidarBeamMin) * 180.0 / std::numbers::pi
            : 0.0;
        const double maxSwingDeg = std::min(80.0,
            std::atan2(heightRange, m_lidarBeamMin) * 180.0 / std::numbers::pi);
        const double uncoveredDeg = std::max(0.0, maxSwingDeg - fovOuterDeg);
        m_numAltSteps = static_cast<int>(std::ceil(uncoveredDeg / 15.0));
        // Use at least 4 altitude steps so intermediate angles (e.g. ±20°, ±40°,
        // ±60°, ±80°) cover floor/ceiling cells at varying horizontal distances.
        m_numAltSteps = std::max(m_numAltSteps, 4);
        m_numAltSteps = std::min(m_numAltSteps, 6);
    } else {
        m_bfsStep     = 50;
        m_elevStep    = 30.0;
        m_numAltSteps = 5;
    }
}

// Run — descend to minHeight, then climb to maxHeight, scanning each level
void MappingAlgorithm::Run()
{
    // Build the list of height levels to visit, covering [minHeight, maxHeight]
    // in m_elevStep increments, always including both endpoints.
    std::vector<double> heights;
    for (double h = static_cast<double>(m_minHeight);
         h <= static_cast<double>(m_maxHeight) + 0.5;
         h += m_elevStep)
    {
        heights.push_back(std::min(h, static_cast<double>(m_maxHeight)));
    }
    if (heights.empty() || heights.back() < static_cast<double>(m_maxHeight) - 0.5)
        heights.push_back(static_cast<double>(m_maxHeight));

    // Start from the closest height to the drone's current Z so we minimise
    // the initial travel, then visit the rest in ascending order.
    const double startZ = m_drone.GetLocation().z.numerical_value_in(cm);
    std::size_t startIdx = 0;
    double bestDist = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < heights.size(); ++i) {
        const double d = std::abs(heights[i] - startZ);
        if (d < bestDist) { bestDist = d; startIdx = i; }
    }

    // Descend from startIdx to 0, then ascend to the top.
    auto visitHeight = [&](double h) {
        m_currentHeightLevel = static_cast<int>(h);
        // Move toward target height; if the floor or ceiling blocks us we stop
        // just short of it and scan from there — altitude beams still cover the
        // surface we couldn't physically reach.
        AdjustHeight(h * cm);
        ScanAndUpdate();
        ExploreAtCurrentHeight();
    };

    visitHeight(heights[startIdx]);
    for (int i = static_cast<int>(startIdx) - 1; i >= 0; --i)
        visitHeight(heights[static_cast<std::size_t>(i)]);
    for (std::size_t i = startIdx + 1; i < heights.size(); ++i)
        visitHeight(heights[i]);
}

// ScanAndUpdate — rotate 360° scanning at each slice; at each heading sweep
// altitude from below to above for full vertical coverage.
//
// Slice count: the FOVC outer circle has 4^(FOVC-1) beams at fine angular
// spacing, but the center beam (which alone covers the inner 0–26° blind spot)
// only rotates at the slice granularity. We match the center-beam step to the
// outer circle's angular density so the two complement each other perfectly.
// Capped at 64 slices (5.625°) to keep runtime reasonable with large FOVC.
void MappingAlgorithm::ScanAndUpdate()
{
    const double maxRotateDeg = m_config
        ? m_config->maxRotate.numerical_value_in(deg)
        : 45.0;

    // Rotation step: we want the center beam to hit every output-grid cell on the
    // furthest wall. Required step = atan2(outputRes, beamMax). This gives ~2.4°
    // for 5cm/120cm — too many slices for practical runtime. We cap at 72 (5°):
    // at 5° the center beam spacing at max range is ~10cm (2 cells), and the
    // FOVC circles fill most of the remaining gaps.
    // NOTE: higher FOVC expands the angular field (wider cone), not center density.
    // The center blind spot (0° to first ring) is the same regardless of FOVC,
    // so numSlices must NOT scale with FOVC — it scales with desired coverage density.
    const double idealStepDeg = std::atan2(m_outputResolution, m_lidarBeamMax)
                                 * 180.0 / std::numbers::pi;
    const std::size_t numSlices = std::min(
        std::size_t{72},
        static_cast<std::size_t>(std::ceil(360.0 / idealStepDeg)));
    const double sliceAngle = 360.0 / static_cast<double>(numSlices);

    // Build altitude sweep angles centred on 0°.
    const double curZ    = m_drone.GetLocation().z.numerical_value_in(cm);
    const double downDeg = std::min(80.0,
        std::atan2(curZ - static_cast<double>(m_minHeight),
                   m_lidarBeamMin) * 180.0 / std::numbers::pi);
    const double upDeg   = std::min(80.0,
        std::atan2(static_cast<double>(m_maxHeight) - curZ,
                   m_lidarBeamMin) * 180.0 / std::numbers::pi);

    std::vector<double> altAngles;
    altAngles.push_back(0.0);
    if (m_numAltSteps > 0) {
        const double stepDown = downDeg / m_numAltSteps;
        const double stepUp   = upDeg   / m_numAltSteps;
        for (int k = 1; k <= m_numAltSteps; ++k) {
            altAngles.push_back(-k * stepDown);
            altAngles.push_back( k * stepUp);
        }
    }

    for (std::size_t slice = 0; slice < numSlices; ++slice) {
        for (double altDeg : altAngles)
            ScanSingleDirection(altDeg);

        double remaining = sliceAngle;
        while (remaining > 0.5) {
            double step = std::min(remaining, maxRotateDeg);
            // Rotation of a sphere never causes collision — see HasClearance comment.
            m_drone.Rotate(step * deg);
            m_currentHeading += step;
            if (m_currentHeading >= 360.0) m_currentHeading -= 360.0;
            remaining -= step;
        }
    }
}

// ScanSingleDirection — perform a single lidar scan at the current heading and given altitude angle,
// and update the map with the results. Mark cells along the beam path as empty until the hit, 
// or until the max range for misses.
void MappingAlgorithm::ScanSingleDirection(double altitudeDeg)
{
    const Orientation scanOri{ 0.0 * deg, altitudeDeg * deg };
    const ScanResults hits     = m_drone.Scan(scanOri);
    const Position3D  dronePos = m_drone.GetLocation();

    const double droneX = dronePos.x.numerical_value_in(cm);
    const double droneY = dronePos.y.numerical_value_in(cm);
    const double droneZ = dronePos.z.numerical_value_in(cm);

    for (const auto& hit : hits) {
        const double d = hit.distance.numerical_value_in(cm);
        if (d <= 0.0) continue;

        const double relHDeg = hit.angle.horizontal.numerical_value_in(deg);
        const double altDeg  = hit.angle.altitude.numerical_value_in(deg);

        const double absHRad = (relHDeg + m_currentHeading) * std::numbers::pi / 180.0;
        const double altRad  = altDeg * std::numbers::pi / 180.0;

        const double cosAlt = std::cos(altRad);
        const double dirX   = cosAlt * std::cos(absHRad);
        const double dirY   = cosAlt * std::sin(absHRad);
        const double dirZ   = std::sin(altRad);

        // traceBeam returns beam_max+1 for genuine misses; a real hit has d <= beam_max.
        const bool isHit = (d <= m_lidarBeamMax);
        if (isHit) {
            m_drone.RecordCell(
                (droneX + d * dirX) * x_extent[cm],
                (droneY + d * dirY) * y_extent[cm],
                (droneZ + d * dirZ) * z_extent[cm],
                MapValue::Occupied);
        }

        // Mark cells along the beam path as empty at output-grid resolution.
        // For misses, stop one step before beam_max to avoid snapping onto a
        // wall cell that was just barely out of range (Bug: diagonal near-miss).
        const double emptyStep = m_outputResolution;
        const double emptyEnd  = isHit ? (d - emptyStep * 0.5)
                                       : (m_lidarBeamMax - emptyStep);
        for (double t = emptyStep; t <= emptyEnd; t += emptyStep) {
            m_drone.RecordCell(
                (droneX + t * dirX) * x_extent[cm],
                (droneY + t * dirY) * y_extent[cm],
                (droneZ + t * dirZ) * z_extent[cm],
                MapValue::Empty);
        }
    }
}

// ExploreAtCurrentHeight — BFS at the current height slice
void MappingAlgorithm::ExploreAtCurrentHeight()
{
    Position3D  curPos  = m_drone.GetLocation();
    GridCell3D  curCell = WorldToGrid(curPos.x, curPos.y, curPos.z);
    m_visited.insert(curCell);

    // Expand neighbours of a cell onto the frontier
    const int S = m_bfsStep;
    auto expand = [&](const GridCell3D& cell) {
        for (const GridCell3D& nb : std::vector<GridCell3D>{
                {cell.x + S, cell.y,     cell.z},
                {cell.x - S, cell.y,     cell.z},
                {cell.x,     cell.y + S, cell.z},
                {cell.x,     cell.y - S, cell.z}}) {
            if (!m_visited.count(nb) && IsWalkable(nb) && HasClearance(nb))
                m_frontier.push(nb);
        }
    };

    expand(curCell);

    while (!m_frontier.empty()) {
        GridCell3D target = m_frontier.front();
        m_frontier.pop();

        if (m_visited.count(target)) continue;

        std::vector<GridCell3D> path = FindPath(target);
        if (path.empty()) continue;

        for (std::size_t i = 1; i < path.size(); ++i) {
            if (!MoveToCell(path[i])) return;
            m_visited.insert(path[i]);
            ScanAndUpdate();
            expand(path[i]);
        }
    }
}

// ElevateToNextHeight (unused in new Run, kept for compatibility)
bool MappingAlgorithm::ElevateToNextHeight()
{
    m_currentHeightLevel += 30;
    return AdjustHeight(m_currentHeightLevel * cm);
}

// WorldToGrid / GridToWorld
GridCell3D MappingAlgorithm::WorldToGrid(XLength x, YLength y, ZLength z) const
{
    return {
        static_cast<int>(x.numerical_value_in(cm) / m_resolution),
        static_cast<int>(y.numerical_value_in(cm) / m_resolution),
        static_cast<int>(z.numerical_value_in(cm) / m_resolution)
    };
}

Position3D MappingAlgorithm::GridToWorld(const GridCell3D& cell) const
{
    return {
        cell.x * m_resolution * x_extent[cm],
        cell.y * m_resolution * y_extent[cm],
        cell.z * m_resolution * z_extent[cm]
    };
}

// IsWalkable / HasClearance 
// check if the cell is free of obstacles and within boundaries, 
// and if the drone can fit there with its configured passage dimensions.
bool MappingAlgorithm::IsWalkable(const GridCell3D& cell) const
{
    Position3D wp  = GridToWorld(cell);
    MapValue   val = m_drone.QueryCell(wp.x, wp.y, wp.z);
    return val != MapValue::Occupied && val != MapValue::NotReachable;
}

bool MappingAlgorithm::HasClearance(const GridCell3D& to) const
{
    if (!m_config) return true;

    // The drone is a perfect sphere (assignment note), so its clearance radius
    // is the same in every direction. Use the minimum of the configured passage
    // dimensions as the sphere radius — that is the tightest constraint.
    const double r = std::min(
        std::min(m_config->minPassWidth .numerical_value_in(cm),
                 m_config->minPassHeight.numerical_value_in(cm)),
        m_config->minPassLength.numerical_value_in(cm)) / 2.0;

    Position3D wp = GridToWorld(to);
    const double tx = wp.x.numerical_value_in(cm);
    const double ty = wp.y.numerical_value_in(cm);
    const double tz = wp.z.numerical_value_in(cm);

    auto occupied = [&](double cx, double cy, double cz) {
        return m_drone.QueryCell(cx * x_extent[cm],
                                 cy * y_extent[cm],
                                 cz * z_extent[cm]) == MapValue::Occupied;
    };

    // Check sphere radius in all 6 axis-aligned directions uniformly.
    return !occupied(tx + r, ty,     tz    )
        && !occupied(tx - r, ty,     tz    )
        && !occupied(tx,     ty + r, tz    )
        && !occupied(tx,     ty - r, tz    )
        && !occupied(tx,     ty,     tz + r)
        && !occupied(tx,     ty,     tz - r);
}

// FindPath — A* - Find a path from the drone's current position to the target cell,
// avoiding occupied and unreachable cells. Returns an empty path if no valid path exists.
std::vector<GridCell3D> MappingAlgorithm::FindPath(const GridCell3D& target)
{
    Position3D startPos = m_drone.GetLocation();
    GridCell3D start    = WorldToGrid(startPos.x, startPos.y, startPos.z);

    if (start == target) return {start};

    auto heuristic = [](const GridCell3D& a, const GridCell3D& b) {
        return std::abs(a.x - b.x) + std::abs(a.y - b.y) + std::abs(a.z - b.z);
    };

    using Entry = std::pair<int, GridCell3D>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> openSet;

    std::map<GridCell3D, int>        gScore;
    std::map<GridCell3D, GridCell3D> parent;
    std::set<GridCell3D>             closed;

    gScore[start] = 0;
    parent[start] = start;
    openSet.push({heuristic(start, target), start});

    while (!openSet.empty()) {
        auto [f, current] = openSet.top();
        openSet.pop();

        if (closed.count(current)) continue;
        closed.insert(current);

        if (current == target) {
            std::vector<GridCell3D> path;
            for (GridCell3D node = target; !(node == start); node = parent[node])
                path.push_back(node);
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const GridCell3D& nb : std::vector<GridCell3D>{
                {current.x + m_bfsStep, current.y,             current.z},
                {current.x - m_bfsStep, current.y,             current.z},
                {current.x,             current.y + m_bfsStep, current.z},
                {current.x,             current.y - m_bfsStep, current.z}}) {
            if (closed.count(nb) || !IsWalkable(nb)) continue;

            int tentativeG = gScore[current] + m_bfsStep;
            if (!gScore.count(nb) || tentativeG < gScore[nb]) {
                gScore[nb] = tentativeG;
                parent[nb] = current;
                openSet.push({tentativeG + heuristic(nb, target), nb});
            }
        }
    }

    return {};
}

// MoveToCell - Move the drone to the target cell by first rotating to face it, then advancing in steps,
// and finally adjusting height if needed. Returns false if any movement step fails due to collision.
bool MappingAlgorithm::MoveToCell(const GridCell3D& target)
{
    Position3D wp = GridToWorld(target);

    if (!AdjustHeight(wp.z.numerical_value_in(cm) * cm))
        return false;

    const double maxAdvanceCm = m_config
        ? m_config->maxAdvance.numerical_value_in(cm)
        : 50.0;

    for (;;) {
        if (!RotateToFace(wp.x, wp.y)) return false;

        Position3D cur = m_drone.GetLocation();
        const double dx   = wp.x.numerical_value_in(cm) - cur.x.numerical_value_in(cm);
        const double dy   = wp.y.numerical_value_in(cm) - cur.y.numerical_value_in(cm);
        const double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < 0.1) break;

        const double step = std::min(maxAdvanceCm, dist);
        if (m_drone.Advance(step * cm) == MoveResult::CollisionDetected)
            return false;
    }

    return true;
}

// RotateToFace - Rotate the drone to face the target (targetX, targetY) from its current position.
bool MappingAlgorithm::RotateToFace(XLength targetX, YLength targetY)
{
    Position3D cur = m_drone.GetLocation();
    const double dx = targetX.numerical_value_in(cm) - cur.x.numerical_value_in(cm);
    const double dy = targetY.numerical_value_in(cm) - cur.y.numerical_value_in(cm);

    if (std::abs(dx) < 0.1 && std::abs(dy) < 0.1) return true;

    double desired = std::atan2(dy, dx) * 180.0 / std::numbers::pi;
    if (desired < 0.0) desired += 360.0;

    const double maxRot = m_config
        ? m_config->maxRotate.numerical_value_in(deg)
        : 45.0;

    double diff = desired - m_currentHeading;
    if (diff >  180.0) diff -= 360.0;
    if (diff < -180.0) diff += 360.0;

    while (std::abs(diff) > 1.0) {
        double step = diff > 0
            ? std::min(diff,  maxRot)
            : std::max(diff, -maxRot);

        // The drone is a perfect sphere (assignment note), so rotation never
        // changes its footprint and cannot cause a collision. The check below
        // is kept as a defensive guard against unexpected simulator behaviour,
        // but it should never fire in practice.
        if (m_drone.Rotate(step * deg) == MoveResult::CollisionDetected)
            return false;

        m_currentHeading += step;
        if (m_currentHeading >= 360.0) m_currentHeading -= 360.0;
        if (m_currentHeading <    0.0) m_currentHeading += 360.0;

        diff = desired - m_currentHeading;
        if (diff >  180.0) diff -= 360.0;
        if (diff < -180.0) diff += 360.0;
    }

    return true;
}

// AdjustHeight - Adjust the drone's height to the targetHeight by elevating in steps, checking for collisions.
bool MappingAlgorithm::AdjustHeight(PhysicalLength targetHeight)
{
    const double maxElev = m_config
        ? m_config->maxElevate.numerical_value_in(cm)
        : 30.0;

    while (true) {
        const double curZ  = m_drone.GetLocation().z.numerical_value_in(cm);
        const double targZ = targetHeight.numerical_value_in(cm);
        const double diff  = targZ - curZ;

        if (std::abs(diff) < 0.1) break;

        const double step = diff > 0
            ? std::min(diff,  maxElev)
            : std::max(diff, -maxElev);

        if (m_drone.Elevate(step * cm) == MoveResult::CollisionDetected)
            return false;
    }
    return true;
}

} // namespace dm