#include "DriftController.h"

namespace {
float positiveFloat(float value) { return value < 0.0f ? -value : value; }
} // namespace

DriftController::DriftController()
    : positionKp_(0.0f), speedKp_(0.0f), maxCorrectionDegrees_(0.0f),
      invertCorrection_(false), neutralPositionDegrees_(0.0f),
      lastCorrectionDegrees_(0.0f) {}

void DriftController::configure(float positionKp, float speedKp,
                                float maxCorrectionDegrees,
                                bool invertCorrection) {
  positionKp_ = positionKp;
  speedKp_ = speedKp;
  maxCorrectionDegrees_ = positiveFloat(maxCorrectionDegrees);
  invertCorrection_ = invertCorrection;
}

void DriftController::reset(const WheelFeedback& feedback) {
  neutralPositionDegrees_ = feedback.averagePositionDegrees;
  lastCorrectionDegrees_ = 0.0f;
}

float DriftController::update(const WheelFeedback& feedback) {
  const float positionError =
      feedback.averagePositionDegrees - neutralPositionDegrees_;
  float correction =
      -((positionKp_ * positionError) + (speedKp_ * feedback.averageSpeedRpm));
  if (invertCorrection_) {
    correction = -correction;
  }

  lastCorrectionDegrees_ = clampCorrection(correction);
  return lastCorrectionDegrees_;
}

float DriftController::lastCorrectionDegrees() const {
  return lastCorrectionDegrees_;
}

float DriftController::clampCorrection(float value) const {
  if (value > maxCorrectionDegrees_) {
    return maxCorrectionDegrees_;
  }
  if (value < -maxCorrectionDegrees_) {
    return -maxCorrectionDegrees_;
  }
  return value;
}
