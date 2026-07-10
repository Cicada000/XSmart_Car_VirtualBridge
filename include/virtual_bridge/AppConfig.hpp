#pragma once

#include "RobotPositionJson.hpp"
#include "VehicleModel.hpp"

#include <string>
#include <vector>

namespace virtual_bridge {

struct AppConfig {
    std::string configPath = "config/virtual_bridge.json";
    std::string controlBindIp = "127.0.0.1";
    int controlPort = 8899;
    std::string udpHost = "127.0.0.1";
    int udpPort = 9005;
    double sendHz = 30.0;
    double initialWorldXmm = 315.0;
    double initialWorldYmm = 1077.0;
    double initialHeadingDeg = 350.0;
    bool quiet = false;
    VehicleParameters vehicle;
    RobotPositionConfig robotPosition;
};

std::string defaultConfigPath();
bool appConfigFileExists(const std::string& path);
AppConfig loadAppConfigFile(const std::string& path);
void applyCommandLineOverrides(AppConfig& config, const std::vector<std::string>& args);
void applyCommandLineOverrides(AppConfig& config, int argc, const char* const* argv);
std::string findConfigPathArgument(const std::vector<std::string>& args,
                                   const std::string& fallback,
                                   bool& explicitPath);

} // namespace virtual_bridge
