#include "IO/ConfigParser.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <algorithm>

namespace dm {

static std::string Trim(const std::string& s)
{
    const auto a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return {};
    const auto b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

using KVMap = std::vector<std::pair<std::string, std::string>>;

static KVMap LoadKV(const std::filesystem::path& path,
                    ErrorLogger& logger,
                    bool& opened)
{
    KVMap result;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Error: cannot open config file: " << path << "\n";
        opened = false;
        return result;
    }
    opened = true;

    int lineNum = 0;
    std::string line;
    while (std::getline(file, line)) {
        ++lineNum;
        const std::string t = Trim(line);
        if (t.empty() || t[0] == '#') continue;

        const auto eq = t.find('=');
        if (eq == std::string::npos) {
            logger.LogError(path.filename().string() + " line " +
                       std::to_string(lineNum) + ": no '=' found — skipped");
            continue;
        }
        result.emplace_back(Trim(t.substr(0, eq)), Trim(t.substr(eq + 1)));
    }
    return result;
}

static std::string Find(const KVMap& kv, const std::string& key)
{
    for (const auto& [k, v] : kv) {
        if (k == key) return v;
    }
    return {};
}

static double GetDouble(const KVMap& kv, const std::string& key,
                        double defaultVal, const std::string& file,
                        ErrorLogger& logger)
{
    const std::string val = Find(kv, key);
    if (val.empty()) {
        logger.LogError(file + ": missing key '" + key +
                   "' — using default " + std::to_string(defaultVal));
        return defaultVal;
    }
    try {
        return std::stod(val);
    } catch (...) {
        logger.LogError(file + ": bad value for '" + key + "' ('" + val +
                   "') — using default " + std::to_string(defaultVal));
        return defaultVal;
    }
}

static PhysicalLength GetCm(const KVMap& kv, const std::string& key,
                             PhysicalLength defaultVal, const std::string& file,
                             ErrorLogger& logger)
{
    const double raw = GetDouble(kv, key, defaultVal.numerical_value_in(cm), file, logger);
    return raw * cm;
}

static HorizontalAngle GetDeg(const KVMap& kv, const std::string& key,
                               HorizontalAngle defaultVal, const std::string& file,
                               ErrorLogger& logger)
{
    const double raw = GetDouble(kv, key, defaultVal.numerical_value_in(deg), file, logger);
    return raw * deg;
}

static std::size_t GetSizeT(const KVMap& kv, const std::string& key,
                             std::size_t defaultVal, const std::string& file,
                             ErrorLogger& logger)
{
    const std::string val = Find(kv, key);
    if (val.empty()) {
        logger.LogError(file + ": missing key '" + key +
                   "' — using default " + std::to_string(defaultVal));
        return defaultVal;
    }
    try {
        const long long v = std::stoll(val);
        if (v < 0) throw std::range_error("negative");
        return static_cast<std::size_t>(v);
    } catch (...) {
        logger.LogError(file + ": bad value for '" + key + "' ('" + val +
                   "') — using default " + std::to_string(defaultVal));
        return defaultVal;
    }
}

// Parse a polygon from a string like: "(0,0),(100,0),(100,100),(0,100)"
// or "0 0, 100 0, 100 100, 0 100"  (whitespace-separated pairs)
// Returns an empty polygon and logs an error if unparseable.
static std::vector<std::pair<double,double>>
ParsePolygon(const std::string& raw, const std::string& file, ErrorLogger& logger)
{
    std::vector<std::pair<double,double>> result;
    if (raw.empty()) {
        // No polygon provided — not necessarily an error (handled by caller)
        return result;
    }

    // Replace parentheses and commas with spaces for uniform tokenisation
    std::string cleaned;
    cleaned.reserve(raw.size());
    for (char c : raw) {
        if (c == '(' || c == ')') cleaned += ' ';
        else if (c == ',')        cleaned += ' ';
        else                      cleaned += c;
    }

    std::istringstream ss(cleaned);
    std::vector<double> nums;
    double n;
    while (ss >> n) nums.push_back(n);

    if (nums.size() < 6 || nums.size() % 2 != 0) {
        logger.LogError(file + ": boundary_polygon needs at least 3 x,y pairs — got '" + raw + "'");
        return result;
    }

    for (std::size_t i = 0; i + 1 < nums.size(); i += 2)
        result.emplace_back(nums[i], nums[i+1]);

    return result;
}

bool ParseDroneConfig(const std::filesystem::path& path,
                      DroneConfig&                 drone_config,
                      ErrorLogger&                 logger)
{
    bool opened = false;
    const KVMap kv = LoadKV(path, logger, opened);
    if (!opened) return false;

    const std::string f = path.filename().string();

    drone_config.minPassWidth       = GetCm (kv, "min_pass_width_cm",        drone_config.minPassWidth,       f, logger);
    drone_config.minPassLength      = GetCm (kv, "min_pass_length_cm",       drone_config.minPassLength,      f, logger);
    drone_config.minPassHeight      = GetCm (kv, "min_pass_height_cm",       drone_config.minPassHeight,      f, logger);
    drone_config.lidarMinBeam       = GetCm (kv, "lidar_beam_min_cm",        drone_config.lidarMinBeam,       f, logger);
    drone_config.lidarMaxBeam       = GetCm (kv, "lidar_beam_max_cm",        drone_config.lidarMaxBeam,       f, logger);
    drone_config.lidarCircleSpacing = GetCm (kv, "lidar_circle_spacing_cm",  drone_config.lidarCircleSpacing, f, logger);
    drone_config.lidarFOVC          = GetSizeT(kv, "lidar_fov_circles",      drone_config.lidarFOVC,          f, logger);
    drone_config.maxRotate          = GetDeg(kv, "max_rotate_deg",           drone_config.maxRotate,          f, logger);
    drone_config.maxAdvance         = GetCm (kv, "max_advance_cm",           drone_config.maxAdvance,         f, logger);
    drone_config.maxElevate         = GetCm (kv, "max_elevate_cm",           drone_config.maxElevate,         f, logger);

    // Apply safe defaults for any zero values that would cause problems
    if (drone_config.maxRotate.numerical_value_in(deg) <= 0.0) {
        logger.LogError(f + ": max_rotate_deg must be > 0, using 45");
        drone_config.maxRotate = 45.0 * deg;
    }
    if (drone_config.maxAdvance.numerical_value_in(cm) <= 0.0) {
        logger.LogError(f + ": max_advance_cm must be > 0, using 10");
        drone_config.maxAdvance = 10.0 * cm;
    }
    if (drone_config.maxElevate.numerical_value_in(cm) <= 0.0) {
        logger.LogError(f + ": max_elevate_cm must be > 0, using 10");
        drone_config.maxElevate = 10.0 * cm;
    }
    if (drone_config.lidarFOVC == 0) {
        logger.LogError(f + ": lidar_fov_circles must be > 0, using 1");
        drone_config.lidarFOVC = 1;
    }
    return true;
}

bool ParseMissionConfig(const std::filesystem::path& path,
                        MissionConfig&               mission_config,
                        ErrorLogger&                 logger)
{
    bool opened = false;
    const KVMap kv = LoadKV(path, logger, opened);
    if (!opened) return false;

    const std::string f = path.filename().string();

    mission_config.boundaryPolygon = ParsePolygon(Find(kv, "boundary_polygon"), f, logger);
    mission_config.minHeight       = GetDouble(kv, "min_height_cm",  mission_config.minHeight.numerical_value_in(cm),  f, logger) * z_extent[cm];
    mission_config.maxHeight       = GetDouble(kv, "max_height_cm",  mission_config.maxHeight.numerical_value_in(cm),  f, logger) * z_extent[cm];
    mission_config.outputResXY      = GetDouble(kv, "output_resolution_xy", mission_config.outputResXY, f, logger);
    mission_config.outputResH       = GetDouble(kv, "output_resolution_h",  mission_config.outputResH,  f, logger);
    mission_config.mapResolutionCm  = GetDouble(kv, "map_resolution_cm",    mission_config.mapResolutionCm, f, logger);
    if (mission_config.mapResolutionCm <= 0.0) {
        std::cerr << "Error: map_resolution_cm must be > 0 (got "
                  << mission_config.mapResolutionCm << "). Aborting.\n";
        return false;
    }
    mission_config.startX          = GetDouble(kv, "start_x_cm",      mission_config.startX.numerical_value_in(cm),      f, logger) * x_extent[cm];
    mission_config.startY          = GetDouble(kv, "start_y_cm",      mission_config.startY.numerical_value_in(cm),      f, logger) * y_extent[cm];
    mission_config.startHeight     = GetDouble(kv, "start_height_cm", mission_config.startHeight.numerical_value_in(cm), f, logger) * z_extent[cm];

    // Validate height bounds
    if (mission_config.maxHeight.numerical_value_in(cm) <= mission_config.minHeight.numerical_value_in(cm)) {
        logger.LogError(f + ": max_height_cm must be > min_height_cm, using min+300");
        mission_config.maxHeight = mission_config.minHeight + 300.0 * z_extent[cm];
    }

    return true;
}

} // namespace dm