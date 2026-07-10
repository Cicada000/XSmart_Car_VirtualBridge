#include "virtual_bridge/VehicleModel.hpp"

#include <algorithm>
#include <cmath>

namespace virtual_bridge {
namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double low, double high) {
    return std::max(low, std::min(value, high));
}

double moveToward(double current, double target, double maxDelta) {
    if (maxDelta <= 0.0) return target;
    if (target > current) return std::min(target, current + maxDelta);
    return std::max(target, current - maxDelta);
}

} // namespace

double degreesToRadians(double degrees) {
    return degrees * kPi / 180.0;
}

double radiansToDegrees(double radians) {
    return radians * 180.0 / kPi;
}

double normalizeRadians(double radians) {
    while (radians > kPi) radians -= 2.0 * kPi;
    while (radians < -kPi) radians += 2.0 * kPi;
    return radians;
}

double servoPulseToSteeringRad(std::uint16_t pulseUs, const VehicleParameters& params) {
    const double span = params.servoPulseSpanUs > 1e-9 ? params.servoPulseSpanUs : 2000.0;
    const double rawDeg = (static_cast<double>(pulseUs) - static_cast<double>(params.servoMidPulseUs)) *
                          180.0 / span * params.steeringSign;
    const double limitedDeg = clamp(rawDeg, -std::abs(params.maxSteeringDeg), std::abs(params.maxSteeringDeg));
    return degreesToRadians(limitedDeg);
}

VehicleModel::VehicleModel(VehicleParameters params) : params_(params) {
    resetFromPosePoint(0.0, 0.0, 0.0);
}

void VehicleModel::resetFromPosePoint(double worldXM, double worldYM, double headingDeg) {
    state_ = {};
    state_.headingRad = normalizeRadians(degreesToRadians(headingDeg));
    state_.worldXM = worldXM - params_.posePointForwardOffsetM * std::cos(state_.headingRad);
    state_.worldYM = worldYM - params_.posePointForwardOffsetM * std::sin(state_.headingRad);
}

void VehicleModel::update(const ControlCommand& command, double dtS) {
    if (dtS <= 0.0) return;
    if (params_.maxDtS > 0.0) dtS = std::min(dtS, params_.maxDtS);

    double targetSpeed = static_cast<double>(command.speedMps) * params_.speedScale;
    if (params_.clampNegativeSpeed) targetSpeed = std::max(0.0, targetSpeed);

    double desiredSpeed = targetSpeed;
    if (params_.speedTimeConstantS > 1e-9) {
        const double alpha = 1.0 - std::exp(-dtS / params_.speedTimeConstantS);
        desiredSpeed = state_.speedMps + (targetSpeed - state_.speedMps) * alpha;
    }
    if (params_.maxAccelMps2 > 0.0) {
        state_.speedMps = moveToward(state_.speedMps, desiredSpeed, params_.maxAccelMps2 * dtS);
    } else {
        state_.speedMps = desiredSpeed;
    }

    const double targetSteering = servoPulseToSteeringRad(command.servoPulseUs, params_);
    if (params_.servoSecPer60Deg > 1e-9) {
        const double servoRateRadPerS = degreesToRadians(60.0 / params_.servoSecPer60Deg);
        state_.steeringRad = moveToward(state_.steeringRad, targetSteering, servoRateRadPerS * dtS);
    } else {
        state_.steeringRad = targetSteering;
    }

    const double wheelbase = std::max(1e-6, params_.wheelbaseM);
    const double yawRate = state_.speedMps / wheelbase * std::tan(state_.steeringRad);
    state_.worldXM += state_.speedMps * std::cos(state_.headingRad) * dtS;
    state_.worldYM += state_.speedMps * std::sin(state_.headingRad) * dtS;
    state_.headingRad = normalizeRadians(state_.headingRad + yawRate * dtS);
}

PosePoint VehicleModel::posePoint() const {
    return {
        state_.worldXM + params_.posePointForwardOffsetM * std::cos(state_.headingRad),
        state_.worldYM + params_.posePointForwardOffsetM * std::sin(state_.headingRad),
        radiansToDegrees(state_.headingRad)
    };
}

RearAxlePose VehicleModel::rearAxlePose() const {
    return state_;
}

} // namespace virtual_bridge
