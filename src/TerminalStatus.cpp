#include "virtual_bridge/TerminalStatus.hpp"

#include <iomanip>
#include <sstream>

namespace virtual_bridge {

std::string buildStatusPanel(const TerminalStatus& status) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "VirtualBridge\n";
    out << "physics_mode: " << (status.physicsEnabled ? "enhanced" : "legacy") << "\n";
    out << "config: " << status.configPath << "\n";
    out << "control_tcp: " << status.controlBindIp << ":" << status.controlPort
        << " " << (status.controlConnected ? "connected" : "waiting") << "\n";
    out << "udp_robot_position: " << status.udpHost << ":" << status.udpPort
        << " @ " << status.sendHz << " Hz\n";
    out << "pose_point_m: x=" << status.pose.worldXM
        << " y=" << status.pose.worldYM
        << " heading=" << status.pose.headingDeg << " deg\n";
    out << "rear: speed=" << status.rear.speedMps
        << " m/s steering=" << radiansToDegrees(status.rear.steeringRad) << " deg\n";
    out << "command: speed=" << static_cast<double>(status.command.speedMps)
        << " m/s servo=" << status.command.servoPulseUs
        << " frames=" << status.commandFrames << "\n";
    out << "udp_send_errors: " << status.udpSendErrors << "\n";
    out << "Press 'R' to reset pose, Ctrl+C to exit.\n";
    return out.str();
}

std::string buildTuiFrame(const TerminalStatus& status, bool firstFrame) {
    std::ostringstream out;
    if (firstFrame) {
        out << "\x1b[?25l\x1b[2J";
    }
    out << "\x1b[H" << buildStatusPanel(status) << "\x1b[J";
    return out.str();
}

std::string buildLegacyStatusLine(const TerminalStatus& status) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "[pose] point_x_m=" << status.pose.worldXM
        << " point_y_m=" << status.pose.worldYM
        << " heading_deg=" << status.pose.headingDeg
        << " rear_speed_mps=" << status.rear.speedMps
        << " steering_deg=" << radiansToDegrees(status.rear.steeringRad)
        << " command_frames=" << status.commandFrames
        << " control=" << (status.controlConnected ? "connected" : "waiting")
        << " udp_send_errors=" << status.udpSendErrors << "\n";
    return out.str();
}

std::string buildTuiShutdownFrame() {
    return "\x1b[?25h\n";
}

} // namespace virtual_bridge
