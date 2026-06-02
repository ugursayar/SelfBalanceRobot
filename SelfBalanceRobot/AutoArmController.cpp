#include "AutoArmController.h"

#include <math.h>

AutoArmController::AutoArmController()
    : angleWindowDegrees_(0.0f),
      maxRateDegPerSec_(0.0f),
      stillMillis_(0),
      targetBalancePointDegrees_(0.0f),
      candidateStartMillis_(0),
      suppressedUntilMillis_(0),
      hasCandidate_(false),
      suppressionActive_(false) {}

void AutoArmController::configure(float angleWindowDegrees,
                                  float maxRateDegPerSec,
                                  unsigned long stillMillis) {
  angleWindowDegrees_ = fabsf(angleWindowDegrees);
  maxRateDegPerSec_ = fabsf(maxRateDegPerSec);
  stillMillis_ = stillMillis;
  reset();
}

void AutoArmController::setTargetBalancePoint(float balancePointDegrees) {
  targetBalancePointDegrees_ = balancePointDegrees;
  reset();
}

void AutoArmController::reset() {
  candidateStartMillis_ = 0;
  hasCandidate_ = false;
}

void AutoArmController::suppressUntil(unsigned long nowMillis,
                                      unsigned long cooldownMillis) {
  suppressedUntilMillis_ = nowMillis + cooldownMillis;
  suppressionActive_ = true;
  reset();
}

float AutoArmController::angleErrorDegrees(const SensorFrame& frame) const {
  return frame.angleDegrees - targetBalancePointDegrees_;
}

bool AutoArmController::update(const SensorFrame& frame) {
  if (updateSuppression(frame.nowMillis)) {
    reset();
    return false;
  }

  const float angleErrorDegrees = this->angleErrorDegrees(frame);
  const bool still =
      frame.gyroFresh && fabsf(angleErrorDegrees) <= angleWindowDegrees_ &&
      fabsf(frame.angleRateDegPerSec) <= maxRateDegPerSec_;

  if (!still) {
    reset();
    return false;
  }

  if (!hasCandidate_) {
    candidateStartMillis_ = frame.nowMillis;
    hasCandidate_ = true;
  }

  return frame.nowMillis - candidateStartMillis_ >= stillMillis_;
}

bool AutoArmController::updateSuppression(unsigned long nowMillis) {
  if (!suppressionActive_) {
    return false;
  }

  if (static_cast<long>(nowMillis - suppressedUntilMillis_) < 0) {
    return true;
  }

  suppressionActive_ = false;
  return false;
}
