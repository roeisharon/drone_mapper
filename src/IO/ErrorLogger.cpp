#include "IO/ErrorLogger.h"
#include <fstream>
#include <iostream>

namespace dm {

void ErrorLogger::LogError(const std::string& message)
{
    errors.push_back(message);
}

bool ErrorLogger::HasErrors() const
{
    return !errors.empty();
}

bool ErrorLogger::FlushToFile(const std::filesystem::path& dir) const
{
    if (errors.empty()) {
        return true;
    }

    const std::filesystem::path outPath = dir / "input_errors.txt";
    std::ofstream file(outPath);
    if (!file.is_open()) {
        std::cerr << "Warning: could not write input_errors.txt to "
                  << outPath << "\n";
        return false;
    }

    for (const auto& msg : errors) {
        file << msg << "\n";
    }
    return true;
}

} // namespace dm