#pragma once

#include "ControlFrame.hpp"

#include <cstdint>

namespace virtual_bridge {

struct PhysicsEnhancementsConfig {
    bool enabled = false;
    double vehicleMassKg = 1.25;
    double minStartSpeedMps = 0.06;
    double coastingDecelMps2 = 0.8;
    int servoTrimUs = 0;
    double servoDeadbandUs = 12.0;
    double sensorLatencyMs = 40.0;
    double positionNoiseM = 0.002;
    double yawNoiseDeg = 0.3;
};

struct VehicleParameters {
    double wheelbaseM = 0.20;
    double rearTrackM = 0.155;
    double posePointForwardOffsetM = 0.075;
    int servoMidPulseUs = 1500;
    double servoPulseSpanUs = 2000.0;
    double maxSteeringDeg = 36.0;
    double steeringSign = 1.0;
    double servoSecPer60Deg = 0.16;
    double speedScale = 1.0;
    double speedTimeConstantS = 0.12;
    double maxAccelMps2 = 3.0;
    bool clampNegativeSpeed = true;
    double maxDtS = 0.20;
    PhysicsEnhancementsConfig physics;
};

struct PosePoint {
    double worldXM = 0.0;
    double worldYM = 0.0;
    double headingDeg = 0.0;
};

struct RearAxlePose {
    double worldXM = 0.0;
    double worldYM = 0.0;
    double headingRad = 0.0;
    double speedMps = 0.0;
    double steeringRad = 0.0;
};

double degreesToRadians(double degrees);
double radiansToDegrees(double radians);
double normalizeRadians(double radians);
double servoPulseToSteeringRad(std::uint16_t pulseUs, const VehicleParameters& params);

class VehicleModel {
public:
    explicit VehicleModel(VehicleParameters params = {});

    void resetFromPosePoint(double worldXM, double worldYM, double headingDeg);
    void update(const ControlCommand& command, double dtS);

    PosePoint posePoint() const;
    RearAxlePose rearAxlePose() const;

private:
    VehicleParameters params_;
    RearAxlePose state_;
};

} // namespace virtual_bridge
