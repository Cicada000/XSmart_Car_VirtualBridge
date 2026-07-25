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
    double effectiveMid = static_cast<double>(params.servoMidPulseUs);

    if (params.physics.enabled) {
        effectiveMid += static_cast<double>(params.physics.servoTrimUs);
        const double diffUs = static_cast<double>(pulseUs) - effectiveMid;
        if (std::abs(diffUs) <= params.physics.servoDeadbandUs) {
            return 0.0;
        }
        const double sign = diffUs > 0 ? 1.0 : -1.0;
        const double activeDiff = (std::abs(diffUs) - params.physics.servoDeadbandUs) * sign;
        const double rawDeg = activeDiff * 180.0 / span * params.steeringSign;
        const double limitedDeg = clamp(rawDeg, -std::abs(params.maxSteeringDeg), std::abs(params.maxSteeringDeg));
        return degreesToRadians(limitedDeg);
    }

    const double rawDeg = (static_cast<double>(pulseUs) - effectiveMid) *
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

    if (!params_.physics.enabled) {
        // --- 沿用经典老版本理想模拟逻辑 ---
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
        return;
    }

    // --- 新版本基于物理增强的模拟逻辑 ---
    double targetSpeed = static_cast<double>(command.speedMps) * params_.speedScale;
    if (params_.clampNegativeSpeed) targetSpeed = std::max(0.0, targetSpeed);

    // 1. 电机起步死区：低于 minStartSpeedMps 时动力输出为 0
    if (std::abs(targetSpeed) < params_.physics.minStartSpeedMps) {
        targetSpeed = 0.0;
    }

    // 2. 区分松油门自然滑行减速与主动驱动/刹车
    if (targetSpeed == 0.0 && state_.speedMps > 0.0) {
        const double decel = params_.physics.coastingDecelMps2 > 0.0 ? params_.physics.coastingDecelMps2 : 0.8;
        state_.speedMps = std::max(0.0, state_.speedMps - decel * dtS);
    } else {
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
    }

    // 3. 舵机中位偏置与死区响应
    const double targetSteering = servoPulseToSteeringRad(command.servoPulseUs, params_);
    if (params_.servoSecPer60Deg > 1e-9) {
        const double servoRateRadPerS = degreesToRadians(60.0 / params_.servoSecPer60Deg);
        state_.steeringRad = moveToward(state_.steeringRad, targetSteering, servoRateRadPerS * dtS);
    } else {
        state_.steeringRad = targetSteering;
    }

    // 4. 中点法 (Midpoint Method / RK2) 轨迹积分
    const double wheelbase = std::max(1e-6, params_.wheelbaseM);
    const double yawRate = state_.speedMps / wheelbase * std::tan(state_.steeringRad);
    const double midHeadingRad = state_.headingRad + 0.5 * yawRate * dtS;

    state_.worldXM += state_.speedMps * std::cos(midHeadingRad) * dtS;
    state_.worldYM += state_.speedMps * std::sin(midHeadingRad) * dtS;
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
