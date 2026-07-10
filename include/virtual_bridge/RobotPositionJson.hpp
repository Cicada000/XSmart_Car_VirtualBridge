#pragma once

#include "VehicleModel.hpp"

#include <cstdint>
#include <string>

namespace virtual_bridge {

enum class CoordinateSource {
    WorldX,
    WorldY
};

struct RobotPositionConfig {
    double heightM = 0.16;
    CoordinateSource posXSource = CoordinateSource::WorldY;
    double posXSign = 1.0;
    CoordinateSource posZSource = CoordinateSource::WorldX;
    double posZSign = 1.0;
    double yawOffsetDeg = 0.0;
    double yawSign = 1.0;
};

std::string buildRobotPositionJson(const PosePoint& pose,
                                   const RobotPositionConfig& config,
                                   std::int64_t emitTimeNs);

} // namespace virtual_bridge
