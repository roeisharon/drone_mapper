#include "IO/MapIO.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>

namespace dm {

// Map file format (plain text):
//
//   # drone_mapper map file
//   x_min=<cm>  x_max=<cm>  y_min=<cm>  y_max=<cm>  z_min=<cm>  z_max=<cm>
//   # x y z value
//   <x_cm> <y_cm> <z_cm> <int_value>
//   ...
//
// int_value: 0=Empty, 1=Occupied, -1=NotMapped, -2=NotReachable
// Lines starting with '#' are comments.  Blank lines are ignored.

static MapValue IntToMapValue(int v)
{
    switch (v) {
        case  0: return MapValue::Empty;
        case  1: return MapValue::Occupied;
        case -1: return MapValue::NotMapped;
        case -2: return MapValue::NotReachable;
        default: return MapValue::NotMapped;
    }
}

static int MapValueToInt(MapValue v) { return static_cast<int>(v); }

ParsedMap ParseMapFile(const std::filesystem::path& path, ErrorLogger& logger)
{
    ParsedMap result;
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Error: cannot open map file: " << path << "\n";
        return result;
    }

    bool boundsRead = false;
    int lineNum = 0;
    std::string line;

    while (std::getline(file, line)) {
        ++lineNum;

        // Strip inline comment
        const auto hashPos = line.find('#');
        if (hashPos != std::string::npos) line = line.substr(0, hashPos);

        // Trim whitespace
        const auto a = line.find_first_not_of(" \t\r\n");
        if (a == std::string::npos) continue;
        line = line.substr(a);

        // Bounds header line: key=value pairs
        if (!boundsRead) {
            if (line.find('=') != std::string::npos) {
                std::istringstream ss(line);
                std::string token;
                bool ok = true;
                while (ss >> token) {
                    const auto eq = token.find('=');
                    if (eq == std::string::npos) { ok = false; break; }
                    const std::string key = token.substr(0, eq);
                    double val = 0.0;
                    try { val = std::stod(token.substr(eq + 1)); }
                    catch (...) { ok = false; break; }

                    if      (key == "x_min") result.bounds.x_min = val * cm;
                    else if (key == "x_max") result.bounds.x_max = val * cm;
                    else if (key == "y_min") result.bounds.y_min = val * cm;
                    else if (key == "y_max") result.bounds.y_max = val * cm;
                    else if (key == "z_min") result.bounds.z_min = val * cm;
                    else if (key == "z_max") result.bounds.z_max = val * cm;
                }
                if (ok) { boundsRead = true; continue; }
            }
        }

        // Data line: x y z value
        std::istringstream ss(line);
        double x, y, z;
        int    val;
        if (!(ss >> x >> y >> z >> val)) {
            logger.LogError(path.filename().string() + " line " +
                            std::to_string(lineNum) + ": expected 'x y z value' — skipped");
            continue;
        }
        MapCell cell;
        cell.x     = x * cm;
        cell.y     = y * cm;
        cell.z     = z * cm;
        cell.value = IntToMapValue(val);
        result.cells.push_back(cell);
    }

    if (!boundsRead) {
        // Derive bounds from cells
        if (!result.cells.empty()) {
            double x_min = 0, x_max = 0, y_min = 1e18, y_max = 0, z_min = 0, z_max = 0;
            for (const auto& c : result.cells) {
                const double cx = c.x.numerical_value_in(cm);
                const double cy = c.y.numerical_value_in(cm);
                const double cz = c.z.numerical_value_in(cm);
                x_min = std::min(x_min, cx); x_max = std::max(x_max, cx);
                y_min = std::min(y_min, cy); y_max = std::max(y_max, cy);
                z_min = std::min(z_min, cz); z_max = std::max(z_max, cz);
            }
            result.bounds.x_min = x_min * cm; result.bounds.x_max = x_max * cm;
            result.bounds.y_min = y_min * cm; result.bounds.y_max = y_max * cm;
            result.bounds.z_min = z_min * cm; result.bounds.z_max = z_max * cm;
            logger.LogError(path.filename().string() + ": bounds header missing — derived from cells");
        } else {
            logger.LogError(path.filename().string() + ": no bounds and no cells found");
            return result;
        }
    }

    result.valid = true;
    return result;
}

bool WriteMapFile(const std::filesystem::path& path,
                  const MapBounds&              bounds,
                  const std::vector<MapCell>&   cells)
{
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "Error: cannot write map file: " << path << "\n";
        return false;
    }

    file << "# drone_mapper map file\n";
    file << "bounds: "
        << "x_min=" << bounds.x_min.numerical_value_in(cm)
        << ", x_max=" << bounds.x_max.numerical_value_in(cm)
        << ", y_min=" << bounds.y_min.numerical_value_in(cm)
        << ", y_max=" << bounds.y_max.numerical_value_in(cm)
        << ", z_min=" << bounds.z_min.numerical_value_in(cm)
        << ", z_max=" << bounds.z_max.numerical_value_in(cm)
        << "\n";
    file << "# x y z value\n";

    for (const auto& c : cells) {
        file << c.x.numerical_value_in(cm) << " "
             << c.y.numerical_value_in(cm) << " "
             << c.z.numerical_value_in(cm) << " "
             << MapValueToInt(c.value) << "\n";
    }
    return file.good();
}

} // namespace dm
