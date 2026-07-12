#include "LqrController.h"

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

LqrController::LqrController()
    : kAngle_(0.0f), kAngleRate_(0.0f), kWheelPosition_(0.0f),
      kWheelSpeed_(0.0f), smallErrorDegrees_(0.0f), smallErrorGainScale_(1.0f),
      rateFilterAlpha_(0.0f),
      filteredMeasuredAngleRateDegreesPerSecond_(0.0f),
      targetAngleDegrees_(0.0f), lastErrorDegrees_(0.0f),
      lastMeasuredAngleRateDegreesPerSecond_(0.0f), outputLimit_(255) {}

void LqrController::setGains(float angleGain, float angleRateGain,
                            float wheelPositionGain, float wheelSpeedGain) {
  kAngle_ = angleGain;
  kAngleRate_ = angleRateGain;
  kWheelPosition_ = wheelPositionGain;
  kWheelSpeed_ = wheelSpeedGain;
}

void LqrController::setGainSchedule(float smallErrorDegrees,
                                    float smallErrorGainScale) {
  smallErrorDegrees_ =
      smallErrorDegrees < 0.0f ? -smallErrorDegrees : smallErrorDegrees;
  smallErrorGainScale_ = clampFloat(smallErrorGainScale, 0.0f, 1.0f);
}

float LqrController::scheduledAngleGain(float angleDeviation) const {
  if (smallErrorDegrees_ <= 0.0f) {
    return kAngle_;
  }
  const float absDeviation =
      angleDeviation < 0.0f ? -angleDeviation : angleDeviation;
  if (absDeviation >= smallErrorDegrees_) {
    return kAngle_;
  }
  const float ramp = absDeviation / smallErrorDegrees_;  // 0 at target, 1 at edge
  const float scale =
      smallErrorGainScale_ + ((1.0f - smallErrorGainScale_) * ramp);
  return kAngle_ * scale;
}

void LqrController::setRateFilter(float filterAlpha) {
  rateFilterAlpha_ = clampFloat(filterAlpha, 0.0f, 0.95f);
}

void LqrController::setTargetAngle(float targetAngleDegrees) {
  targetAngleDegrees_ = targetAngleDegrees;
}

void LqrController::setOutputLimit(int16_t outputLimit) {
  const int32_t widenedLimit = outputLimit;
  const int32_t positiveLimit =
      widenedLimit < 0 ? -widenedLimit : widenedLimit;
  outputLimit_ = positiveLimit > INT16_MAX ? INT16_MAX
                                           : static_cast<int16_t>(positiveLimit);
}

void LqrController::reset() {
  filteredMeasuredAngleRateDegreesPerSecond_ = 0.0f;
  lastErrorDegrees_ = 0.0f;
  lastMeasuredAngleRateDegreesPerSecond_ = 0.0f;
}

int16_t LqrController::update(float measuredAngleDegrees,
                              float measuredAngleRateDegPerSec,
                              float wheelPositionDegrees, float wheelSpeedRpm,
                              float dtSeconds) {
  if (dtSeconds <= 0.0f) {
    return 0;
  }

  // Tilt deviation from the operating point.  lastErrorDegrees_ keeps the PID
  // sign convention (target - measured) so telemetry reads the same quantity
  // regardless of which controller is active.
  const float angleDeviation = measuredAngleDegrees - targetAngleDegrees_;
  lastErrorDegrees_ = -angleDeviation;

  filteredMeasuredAngleRateDegreesPerSecond_ =
      (rateFilterAlpha_ * filteredMeasuredAngleRateDegreesPerSecond_) +
      ((1.0f - rateFilterAlpha_) * measuredAngleRateDegPerSec);
  lastMeasuredAngleRateDegreesPerSecond_ =
      filteredMeasuredAngleRateDegreesPerSecond_;

  // u = K * (x - x*), x = [angle, angleRate, wheelPosition, wheelSpeed].
  // The angle gain is softened near the target by the gain schedule so small
  // corrections produce gentler motor commands.
  const float rawOutput =
      (scheduledAngleGain(angleDeviation) * angleDeviation) +
      (kAngleRate_ * filteredMeasuredAngleRateDegreesPerSecond_) +
      (kWheelPosition_ * wheelPositionDegrees) +
      (kWheelSpeed_ * wheelSpeedRpm);
  const float limitedOutput =
      clampFloat(rawOutput, -static_cast<float>(outputLimit_),
                 static_cast<float>(outputLimit_));

  return static_cast<int16_t>(limitedOutput);
}

float LqrController::lastErrorDegrees() const { return lastErrorDegrees_; }

float LqrController::lastMeasuredAngleRateDegreesPerSecond() const {
  return lastMeasuredAngleRateDegreesPerSecond_;
}
