#pragma once

#include "ControlFrame.hpp"
#include "VehicleModel.hpp"

#include <cstdint>
#include <string>

namespace virtual_bridge {

struct TerminalStatus {
    std::string configPath;
    std::string controlBindIp;
    int controlPort = 0;
    bool controlConnected = false;
    std::string udpHost;
    int udpPort = 0;
    double sendHz = 0.0;
    PosePoint pose;
    RearAxlePose rear;
    ControlCommand command;
    std::uint64_t commandFrames = 0;
    std::uint64_t udpSendErrors = 0;
};

std::string buildStatusPanel(const TerminalStatus& status);
std::string buildTuiFrame(const TerminalStatus& status, bool firstFrame);
std::string buildLegacyStatusLine(const TerminalStatus& status);
std::string buildTuiShutdownFrame();

} // namespace virtual_bridge
