#include "virtual_bridge/ControlFrame.hpp"
#include "virtual_bridge/RobotPositionJson.hpp"
#include "virtual_bridge/VehicleModel.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>

namespace {

constexpr double kPi = 3.14159265358979323846;

std::uint8_t checksumFirstNine(const std::uint8_t* frame) {
    std::uint8_t checksum = 0;
    for (int i = 0; i < 9; ++i) {
        checksum = static_cast<std::uint8_t>(checksum + frame[i]);
    }
    return checksum;
}

void writeFloatLe(std::uint8_t* out, float value) {
    static_assert(sizeof(float) == 4, "float must be 32-bit");
    std::memcpy(out, &value, sizeof(float));
}

void writeUint16Le(std::uint8_t* out, std::uint16_t value) {
    out[0] = static_cast<std::uint8_t>(value & 0xffu);
    out[1] = static_cast<std::uint8_t>((value >> 8u) & 0xffu);
}

void testParsesValidControlFrame() {
    std::uint8_t frame[virtual_bridge::kControlFrameSize]{};
    frame[0] = 0x42;
    frame[1] = 1;
    frame[2] = 10;
    writeFloatLe(frame + 3, 1.25f);
    writeUint16Le(frame + 7, 1900);
    frame[9] = checksumFirstNine(frame);

    const std::optional<virtual_bridge::ControlCommand> command =
        virtual_bridge::parseControlFrame(frame, sizeof(frame));

    assert(command.has_value());
    assert(std::fabs(command->speedMps - 1.25f) < 1e-6f);
    assert(command->servoPulseUs == 1900);
}

void testRejectsBadChecksum() {
    std::uint8_t frame[virtual_bridge::kControlFrameSize]{};
    frame[0] = 0x42;
    frame[1] = 1;
    frame[2] = 10;
    writeFloatLe(frame + 3, 0.5f);
    writeUint16Le(frame + 7, 1500);
    frame[9] = 0;

    const std::optional<virtual_bridge::ControlCommand> command =
        virtual_bridge::parseControlFrame(frame, sizeof(frame));

    assert(!command.has_value());
}

void testServoPulseMapsToFrontWheelAngle() {
    virtual_bridge::VehicleParameters params;
    params.servoMidPulseUs = 1500;
    params.servoPulseSpanUs = 2000;
    params.maxSteeringDeg = 36.0;

    assert(std::fabs(virtual_bridge::servoPulseToSteeringRad(1500, params)) < 1e-12);
    assert(std::fabs(virtual_bridge::servoPulseToSteeringRad(1900, params) - 36.0 * kPi / 180.0) < 1e-12);
    assert(std::fabs(virtual_bridge::servoPulseToSteeringRad(1100, params) + 36.0 * kPi / 180.0) < 1e-12);
}

void testStraightMotionPublishesOffsetPosePoint() {
    virtual_bridge::VehicleParameters params;
    params.wheelbaseM = 0.20;
    params.posePointForwardOffsetM = 0.075;
    params.speedTimeConstantS = 0.0;
    params.maxAccelMps2 = 100.0;
    params.servoSecPer60Deg = 0.0;
    params.maxDtS = 0.0;

    virtual_bridge::VehicleModel model(params);
    model.resetFromPosePoint(1.0, 2.0, 0.0);
    model.update({1.0f, 1500}, 1.0);

    const virtual_bridge::PosePoint pose = model.posePoint();
    assert(std::fabs(pose.worldXM - 2.0) < 1e-9);
    assert(std::fabs(pose.worldYM - 2.0) < 1e-9);
    assert(std::fabs(pose.headingDeg - 0.0) < 1e-9);
}

void testTurningMotionUsesBicycleYawRate() {
    virtual_bridge::VehicleParameters params;
    params.wheelbaseM = 0.20;
    params.posePointForwardOffsetM = 0.075;
    params.speedTimeConstantS = 0.0;
    params.maxAccelMps2 = 100.0;
    params.servoSecPer60Deg = 0.0;
    params.maxSteeringDeg = 36.0;
    params.maxDtS = 0.0;

    virtual_bridge::VehicleModel model(params);
    model.resetFromPosePoint(0.0, 0.0, 0.0);
    model.update({1.0f, 1900}, 0.1);

    const double expectedYaw = (1.0 / 0.20) * std::tan(36.0 * kPi / 180.0) * 0.1;
    assert(std::fabs(model.rearAxlePose().headingRad - expectedYaw) < 1e-9);
}

void testBuildsArucoRobotPositionJson() {
    virtual_bridge::RobotPositionConfig config;
    config.heightM = 0.16;
    config.yawOffsetDeg = 180.0;
    config.yawSign = 1.0;
    config.posXSource = virtual_bridge::CoordinateSource::WorldY;
    config.posZSource = virtual_bridge::CoordinateSource::WorldX;

    const std::string json = virtual_bridge::buildRobotPositionJson(
        {1.25, 2.5, 45.0},
        config,
        123456789);

    assert(json.find("\"type\":\"robot_position\"") != std::string::npos);
    assert(json.find("\"pos\":[2.500000,0.160000,1.250000]") != std::string::npos);
    assert(json.find("\"euler\":[0.0,225.000000,0.0]") != std::string::npos);
    assert(json.find("\"t_aruco_emit_ns\":123456789") != std::string::npos);
}

} // namespace

int main() {
    testParsesValidControlFrame();
    testRejectsBadChecksum();
    testServoPulseMapsToFrontWheelAngle();
    testStraightMotionPublishesOffsetPosePoint();
    testTurningMotionUsesBicycleYawRate();
    testBuildsArucoRobotPositionJson();
    return 0;
}
