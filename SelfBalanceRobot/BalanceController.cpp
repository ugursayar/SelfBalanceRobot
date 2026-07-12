#include "BalanceController.h"

namespace {
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
    : kp_(0.0f), ki_(0.0f), kd_(0.0f),
      smallErrorDegrees_(0.0f), smallErrorGainScale_(1.0f),
      integralLimitDegreesSeconds_(0.0f), targetAngleDegrees_(0.0f),
      integral_(0.0f), previousMeasuredAngleDegrees_(0.0f),
      filteredMeasuredAngleRateDegreesPerSecond_(0.0f),
      lastErrorDegrees_(0.0f),
      lastMeasuredAngleRateDegreesPerSecond_(0.0f), rateFilterAlpha_(0.0f),
      outputLimit_(255),
      hasPreviousMeasuredAngle_(false) {}

void BalanceController::setTunings(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void BalanceController::setIntegralLimit(float integralLimitDegreesSeconds) {
  integralLimitDegreesSeconds_ =
      integralLimitDegreesSeconds < 0.0f ? -integralLimitDegreesSeconds
                                         : integralLimitDegreesSeconds;
}

void BalanceController::setGainSchedule(float smallErrorDegrees,
                                        float smallErrorGainScale) {
  smallErrorDegrees_ =
      smallErrorDegrees < 0.0f ? -smallErrorDegrees : smallErrorDegrees;
  smallErrorGainScale_ = clampFloat(smallErrorGainScale, 0.0f, 1.0f);
}

float BalanceController::scheduledProportionalGain(float error) const {
  if (smallErrorDegrees_ <= 0.0f) {
    return kp_;
  }
  const float absError = error < 0.0f ? -error : error;
  if (absError >= smallErrorDegrees_) {
    return kp_;
  }
  const float ramp = absError / smallErrorDegrees_;  // 0 at upright, 1 at edge
  const float scale = smallErrorGainScale_ + ((1.0f - smallErrorGainScale_) * ramp);
  return kp_ * scale;
}

void BalanceController::setRateFilter(float filterAlpha) {
  rateFilterAlpha_ = clampFloat(filterAlpha, 0.0f, 0.95f);
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
  previousMeasuredAngleDegrees_ = 0.0f;
  filteredMeasuredAngleRateDegreesPerSecond_ = 0.0f;
  lastErrorDegrees_ = 0.0f;
  lastMeasuredAngleRateDegreesPerSecond_ = 0.0f;
  hasPreviousMeasuredAngle_ = false;
}

int16_t BalanceController::update(float measuredAngleDegrees,
                                  float dtSeconds) {
  if (dtSeconds <= 0.0f) {
    return 0;
  }

  const float error = targetAngleDegrees_ - measuredAngleDegrees;
  lastErrorDegrees_ = error;

  if (ki_ != 0.0f) {
    integral_ += error * dtSeconds;
    if (integralLimitDegreesSeconds_ > 0.0f) {
      integral_ = clampFloat(integral_, -integralLimitDegreesSeconds_,
                             integralLimitDegreesSeconds_);
    }
  }

  float measuredAngleRate = 0.0f;
  if (hasPreviousMeasuredAngle_) {
    measuredAngleRate =
        (measuredAngleDegrees - previousMeasuredAngleDegrees_) / dtSeconds;
  }
  filteredMeasuredAngleRateDegreesPerSecond_ =
      (rateFilterAlpha_ * filteredMeasuredAngleRateDegreesPerSecond_) +
      ((1.0f - rateFilterAlpha_) * measuredAngleRate);
  lastMeasuredAngleRateDegreesPerSecond_ =
      filteredMeasuredAngleRateDegreesPerSecond_;

  previousMeasuredAngleDegrees_ = measuredAngleDegrees;
  hasPreviousMeasuredAngle_ = true;

  const float rawOutput =
      -((scheduledProportionalGain(error) * error) + (ki_ * integral_)) +
      (kd_ * filteredMeasuredAngleRateDegreesPerSecond_);
  const float limitedOutput =
      clampFloat(rawOutput, -static_cast<float>(outputLimit_),
                 static_cast<float>(outputLimit_));

  return static_cast<int16_t>(limitedOutput);
}

int16_t BalanceController::update(float measuredAngleDegrees,
                                  float measuredAngleRateDegPerSec,
                                  float dtSeconds) {
  if (dtSeconds <= 0.0f) {
    return 0;
  }

  const float error = targetAngleDegrees_ - measuredAngleDegrees;
  lastErrorDegrees_ = error;

  if (ki_ != 0.0f) {
    integral_ += error * dtSeconds;
    if (integralLimitDegreesSeconds_ > 0.0f) {
      integral_ = clampFloat(integral_, -integralLimitDegreesSeconds_,
                             integralLimitDegreesSeconds_);
    }
  }

  // Use provided raw gyro rate directly — no finite-difference delay.
  filteredMeasuredAngleRateDegreesPerSecond_ =
      (rateFilterAlpha_ * filteredMeasuredAngleRateDegreesPerSecond_) +
      ((1.0f - rateFilterAlpha_) * measuredAngleRateDegPerSec);
  lastMeasuredAngleRateDegreesPerSecond_ =
      filteredMeasuredAngleRateDegreesPerSecond_;

  previousMeasuredAngleDegrees_ = measuredAngleDegrees;
  hasPreviousMeasuredAngle_ = true;

  const float rawOutput =
      -((scheduledProportionalGain(error) * error) + (ki_ * integral_)) +
      (kd_ * filteredMeasuredAngleRateDegreesPerSecond_);
  const float limitedOutput =
      clampFloat(rawOutput, -static_cast<float>(outputLimit_),
                 static_cast<float>(outputLimit_));

  return static_cast<int16_t>(limitedOutput);
}

float BalanceController::lastErrorDegrees() const { return lastErrorDegrees_; }

float BalanceController::lastMeasuredAngleRateDegreesPerSecond() const {
  return lastMeasuredAngleRateDegreesPerSecond_;
}
