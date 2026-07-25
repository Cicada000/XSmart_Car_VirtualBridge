#include "virtual_bridge/ControlFrame.hpp"
#include "virtual_bridge/AppConfig.hpp"
#include "virtual_bridge/RobotPositionJson.hpp"
#include "virtual_bridge/TerminalStatus.hpp"
#include "virtual_bridge/VehicleModel.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <optional>
#include <stdexcept>
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

void testResetFromPosePointRestoresInitialState() {
    virtual_bridge::VehicleParameters params;
    params.wheelbaseM = 0.20;
    params.posePointForwardOffsetM = 0.075;
    params.speedTimeConstantS = 0.0;
    params.maxAccelMps2 = 100.0;
    params.servoSecPer60Deg = 0.0;
    params.maxSteeringDeg = 36.0;
    params.maxDtS = 0.0;

    virtual_bridge::VehicleModel model(params);
    model.resetFromPosePoint(0.315, 1.077, 350.0);
    model.update({1.0f, 1900}, 2.0);

    // After moving, pose is no longer initial
    const virtual_bridge::PosePoint movedPose = model.posePoint();
    assert(std::fabs(movedPose.worldXM - 0.315) > 0.01 || std::fabs(movedPose.worldYM - 1.077) > 0.01);

    // Reset back to initial
    model.resetFromPosePoint(0.315, 1.077, 350.0);
    const virtual_bridge::PosePoint resetPose = model.posePoint();
    assert(std::fabs(resetPose.worldXM - 0.315) < 1e-9);
    assert(std::fabs(resetPose.worldYM - 1.077) < 1e-9);
    assert(std::fabs(virtual_bridge::normalizeRadians(virtual_bridge::degreesToRadians(resetPose.headingDeg - 350.0))) < 1e-9);
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

void testDefaultYawOffsetKeepsModelHeadingAlignedWithPublishedYaw() {
    virtual_bridge::RobotPositionConfig config;

    const std::string json = virtual_bridge::buildRobotPositionJson(
        {1.25, 2.5, 45.0},
        config,
        123456789);

    assert(json.find("\"euler\":[0.0,45.000000,0.0]") != std::string::npos);
}

void testLoadsRuntimeConfigFromJsonFile() {
    const std::string path = "/tmp/virtual_bridge_config_test.json";
    {
        std::ofstream out(path);
        out << R"json({
  // TCP endpoint that XSmart_Car_LineFollower connects to.
  "control": {
    "bind_ip": "0.0.0.0",
    "port": 18899
  },
  // UDP robot_position target.
  "udp": {
    "host": "127.0.0.1",
    "port": 19005,
    "send_hz": 42.0
  },
  "initial_pose": {
    "world_x_mm": 315.0,
    "world_y_mm": 1077.0,
    "heading_deg": 350.0
  },
  "vehicle": {
    "wheelbase_m": 0.21,
    "rear_track_m": 0.16,
    "pose_offset_m": 0.08,
    "servo_mid_us": 1490,
    "servo_span_us": 1980.0,
    "max_steering_deg": 32.0,
    "steering_sign": -1.0,
    "servo_sec_per_60_deg": 0.18,
    "speed_scale": 0.75,
    "speed_tau_s": 0.20,
    "max_accel_mps2": 2.2,
    "clamp_negative_speed": false,
    "max_dt_s": 0.15
  },
  "robot_position": {
    "height_m": 0.17,
    "pos_x_source": "world_x",
    "pos_x_sign": -1.0,
    "pos_z_source": "world_y",
    "pos_z_sign": 1.0,
    "yaw_offset_deg": 5.0,
    "yaw_sign": -1.0
  },
  "runtime": {
    "quiet": true
  },
  "unused_array": [1, true, "x", {"nested": null}]
})json";
    }

    const virtual_bridge::AppConfig config = virtual_bridge::loadAppConfigFile(path);

    assert(config.controlBindIp == "0.0.0.0");
    assert(config.controlPort == 18899);
    assert(config.udpHost == "127.0.0.1");
    assert(config.udpPort == 19005);
    assert(std::fabs(config.sendHz - 42.0) < 1e-12);
    assert(std::fabs(config.initialWorldXmm - 315.0) < 1e-12);
    assert(std::fabs(config.initialWorldYmm - 1077.0) < 1e-12);
    assert(std::fabs(config.initialHeadingDeg - 350.0) < 1e-12);
    assert(std::fabs(config.vehicle.wheelbaseM - 0.21) < 1e-12);
    assert(std::fabs(config.vehicle.rearTrackM - 0.16) < 1e-12);
    assert(std::fabs(config.vehicle.posePointForwardOffsetM - 0.08) < 1e-12);
    assert(config.vehicle.servoMidPulseUs == 1490);
    assert(std::fabs(config.vehicle.servoPulseSpanUs - 1980.0) < 1e-12);
    assert(std::fabs(config.vehicle.maxSteeringDeg - 32.0) < 1e-12);
    assert(std::fabs(config.vehicle.steeringSign + 1.0) < 1e-12);
    assert(std::fabs(config.vehicle.servoSecPer60Deg - 0.18) < 1e-12);
    assert(std::fabs(config.vehicle.speedScale - 0.75) < 1e-12);
    assert(std::fabs(config.vehicle.speedTimeConstantS - 0.20) < 1e-12);
    assert(std::fabs(config.vehicle.maxAccelMps2 - 2.2) < 1e-12);
    assert(!config.vehicle.clampNegativeSpeed);
    assert(std::fabs(config.vehicle.maxDtS - 0.15) < 1e-12);
    assert(std::fabs(config.robotPosition.heightM - 0.17) < 1e-12);
    assert(config.robotPosition.posXSource == virtual_bridge::CoordinateSource::WorldX);
    assert(std::fabs(config.robotPosition.posXSign + 1.0) < 1e-12);
    assert(config.robotPosition.posZSource == virtual_bridge::CoordinateSource::WorldY);
    assert(std::fabs(config.robotPosition.posZSign - 1.0) < 1e-12);
    assert(std::fabs(config.robotPosition.yawOffsetDeg - 5.0) < 1e-12);
    assert(std::fabs(config.robotPosition.yawSign + 1.0) < 1e-12);
    assert(config.quiet);
}

void testLoadsDefaultConfigFile() {
    const virtual_bridge::AppConfig config =
        virtual_bridge::loadAppConfigFile("config/virtual_bridge.json");

    assert(config.controlBindIp == "127.0.0.1");
    assert(config.controlPort == 8899);
    assert(config.udpHost == "127.0.0.1");
    assert(config.udpPort == 9005);
    assert(std::fabs(config.initialWorldXmm - 315.0) < 1e-12);
    assert(std::fabs(config.initialWorldYmm - 1077.0) < 1e-12);
    assert(std::fabs(config.initialHeadingDeg - 350.0) < 1e-12);
    assert(std::fabs(config.vehicle.wheelbaseM - 0.20) < 1e-12);
    assert(std::fabs(config.vehicle.rearTrackM - 0.155) < 1e-12);
    assert(std::fabs(config.vehicle.posePointForwardOffsetM - 0.075) < 1e-12);
}

void testCommandLineOverridesLoadedConfigValues() {
    virtual_bridge::AppConfig config;
    config.udpPort = 19005;
    config.vehicle.wheelbaseM = 0.21;
    config.quiet = false;

    const char* argv[] = {
        "virtual_aruco_bridge",
        "--udp-port", "9005",
        "--wheelbase-m", "0.25",
        "--quiet"
    };

    virtual_bridge::applyCommandLineOverrides(config, 6, argv);

    assert(config.udpPort == 9005);
    assert(std::fabs(config.vehicle.wheelbaseM - 0.25) < 1e-12);
    assert(config.quiet);
}

void testRejectsInvalidCoordinateSourceInConfig() {
    const std::string path = "/tmp/virtual_bridge_bad_config_test.json";
    {
        std::ofstream out(path);
        out << R"json({
  "robot_position": {
    "pos_x_source": "bad_axis"
  }
})json";
    }

    bool threw = false;
    try {
        (void)virtual_bridge::loadAppConfigFile(path);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    assert(threw);
}

void testBuildsStatusPanelForTerminalRefresh() {
    virtual_bridge::TerminalStatus status;
    status.configPath = "config/virtual_bridge.json";
    status.controlBindIp = "127.0.0.1";
    status.controlPort = 8899;
    status.controlConnected = true;
    status.udpHost = "127.0.0.1";
    status.udpPort = 9005;
    status.sendHz = 30.0;
    status.pose = {0.315, 1.077, 350.0};
    status.rear.speedMps = 0.42;
    status.rear.steeringRad = virtual_bridge::degreesToRadians(-12.5);
    status.command.speedMps = 0.5f;
    status.command.servoPulseUs = 1684;
    status.commandFrames = 23;
    status.udpSendErrors = 2;

    const std::string panel = virtual_bridge::buildStatusPanel(status);

    assert(panel.find("VirtualBridge") != std::string::npos);
    assert(panel.find("Press 'R' to reset pose, Ctrl+C to exit.") != std::string::npos);
    assert(panel.find("config/virtual_bridge.json") != std::string::npos);
    assert(panel.find("control_tcp: 127.0.0.1:8899 connected") != std::string::npos);
    assert(panel.find("udp_robot_position: 127.0.0.1:9005 @ 30.000 Hz") != std::string::npos);
    assert(panel.find("pose_point_m: x=0.315 y=1.077 heading=350.000 deg") != std::string::npos);
    assert(panel.find("rear: speed=0.420 m/s steering=-12.500 deg") != std::string::npos);
    assert(panel.find("command: speed=0.500 m/s servo=1684 frames=23") != std::string::npos);
    assert(panel.find("udp_send_errors: 2") != std::string::npos);
}

void testBuildsAnsiTuiFrames() {
    virtual_bridge::TerminalStatus status;
    status.controlBindIp = "127.0.0.1";
    status.controlPort = 8899;
    status.udpHost = "127.0.0.1";
    status.udpPort = 9005;

    const std::string first = virtual_bridge::buildTuiFrame(status, true);
    const std::string next = virtual_bridge::buildTuiFrame(status, false);

    assert(first.rfind("\x1b[?25l\x1b[2J\x1b[H", 0) == 0);
    assert(first.find("\x1b[J") != std::string::npos);
    assert(next.rfind("\x1b[H", 0) == 0);
    assert(next.find("\x1b[2J") == std::string::npos);
    assert(virtual_bridge::buildTuiShutdownFrame() == "\x1b[?25h\n");
}

void testPhysicsEnhancementsToggleAndDeadband() {
    virtual_bridge::VehicleParameters params;
    params.speedTimeConstantS = 0.0;
    params.maxAccelMps2 = 100.0;
    params.servoSecPer60Deg = 0.0;

    // 1. 开关为 false 时（旧模式）：低速指令能正常起步
    params.physics.enabled = false;
    params.physics.minStartSpeedMps = 0.10;
    virtual_bridge::VehicleModel legacyModel(params);
    legacyModel.update({0.05f, 1500}, 0.5);
    assert(legacyModel.rearAxlePose().speedMps > 0.04);

    // 2. 开关为 true 时（物理增强模式）：低于死区速度不动作
    params.physics.enabled = true;
    virtual_bridge::VehicleModel enhancedModel(params);
    enhancedModel.update({0.05f, 1500}, 0.5);
    assert(enhancedModel.rearAxlePose().speedMps == 0.0);

    // 高于死区速度可正常起步
    enhancedModel.update({0.20f, 1500}, 0.5);
    assert(enhancedModel.rearAxlePose().speedMps > 0.15);

    // 3. 验证舵机死区
    params.physics.servoDeadbandUs = 15.0;
    params.physics.servoTrimUs = 10; // 有效中位 1510
    // 脉宽 1520 在死区 [1495, 1525] 范围之内 -> 转向角为 0
    assert(std::fabs(virtual_bridge::servoPulseToSteeringRad(1520, params)) < 1e-9);
}

} // namespace

int main() {
    testParsesValidControlFrame();
    testRejectsBadChecksum();
    testServoPulseMapsToFrontWheelAngle();
    testStraightMotionPublishesOffsetPosePoint();
    testResetFromPosePointRestoresInitialState();
    testTurningMotionUsesBicycleYawRate();
    testBuildsArucoRobotPositionJson();
    testDefaultYawOffsetKeepsModelHeadingAlignedWithPublishedYaw();
    testLoadsRuntimeConfigFromJsonFile();
    testLoadsDefaultConfigFile();
    testCommandLineOverridesLoadedConfigValues();
    testRejectsInvalidCoordinateSourceInConfig();
    testBuildsStatusPanelForTerminalRefresh();
    testBuildsAnsiTuiFrames();
    testPhysicsEnhancementsToggleAndDeadband();
    return 0;
}
