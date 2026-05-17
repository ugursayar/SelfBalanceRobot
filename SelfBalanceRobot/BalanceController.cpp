#include "BalanceController.h"

namespace {
constexpr float kIntegralMin = -50.0f;
constexpr float kIntegralMax = 50.0f;

float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}
} // namespace

BalanceController::BalanceController()
    : kp_(0.0f), ki_(0.0f), kd_(0.0f), smallErrorDegrees_(0.0f),
      smallErrorGainScale_(1.0f), targetAngleDegrees_(0.0f), integral_(0.0f),
      previousError_(0.0f), outputLimit_(255), hasPreviousError_(false) {}

void BalanceController::setTunings(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void BalanceController::setGainSchedule(float smallErrorDegrees,
                                        float smallErrorGainScale) {
  smallErrorDegrees_ = smallErrorDegrees < 0.0f ? -smallErrorDegrees
                                                : smallErrorDegrees;
  smallErrorGainScale_ =
      clampFloat(smallErrorGainScale, 0.0f, 1.0f);
}

void BalanceController::setTargetAngle(float targetAngleDegrees) {
  targetAngleDegrees_ = targetAngleDegrees;
}

void BalanceController::setOutputLimit(int16_t outputLimit) {
  const int32_t widenedLimit = outputLimit;
  const int32_t positiveLimit =
      widenedLimit < 0 ? -widenedLimit : widenedLimit;
  outputLimit_ = positiveLimit > INT16_MAX ? INT16_MAX
                                           : static_cast<int16_t>(positiveLimit);
}

void BalanceController::reset() {
  integral_ = 0.0f;
  previousError_ = 0.0f;
  hasPreviousError_ = false;
}

int16_t BalanceController::update(float measuredAngleDegrees,
                                  float dtSeconds) {
  if (dtSeconds <= 0.0f) {
    return 0;
  }

  const float error = targetAngleDegrees_ - measuredAngleDegrees;

  integral_ += error * dtSeconds;
  integral_ = clampFloat(integral_, kIntegralMin, kIntegralMax);

  float derivative = 0.0f;
  if (hasPreviousError_) {
    derivative = (error - previousError_) / dtSeconds;
  }

  previousError_ = error;
  hasPreviousError_ = true;

  float proportionalGain = kp_;
  if (smallErrorDegrees_ > 0.0f &&
      error >= -smallErrorDegrees_ && error <= smallErrorDegrees_) {
    proportionalGain *= smallErrorGainScale_;
  }

  const float rawOutput = (proportionalGain * error) + (ki_ * integral_) +
                          (kd_ * derivative);
  const float limitedOutput =
      clampFloat(rawOutput, -static_cast<float>(outputLimit_),
                 static_cast<float>(outputLimit_));

  return static_cast<int16_t>(limitedOutput);
}
