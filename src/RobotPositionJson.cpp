#include "virtual_bridge/RobotPositionJson.hpp"

#include <iomanip>
#include <sstream>

namespace virtual_bridge {
namespace {

double coordinateValue(CoordinateSource source, const PosePoint& pose) {
    return source == CoordinateSource::WorldY ? pose.worldYM : pose.worldXM;
}

} // namespace

std::string buildRobotPositionJson(const PosePoint& pose,
                                   const RobotPositionConfig& config,
                                   std::int64_t emitTimeNs) {
    const double x = coordinateValue(config.posXSource, pose) * config.posXSign;
    const double z = coordinateValue(config.posZSource, pose) * config.posZSign;
    const double yaw = config.yawOffsetDeg + config.yawSign * pose.headingDeg;

    std::ostringstream out;
    out << "{\"type\":\"robot_position\",\"pos\":["
        << std::fixed << std::setprecision(6)
        << x << "," << config.heightM << "," << z
        << "],\"euler\":[0.0," << yaw << ",0.0]"
        << ",\"t_aruco_emit_ns\":" << emitTimeNs
        << "}";
    return out.str();
}

} // namespace virtual_bridge
