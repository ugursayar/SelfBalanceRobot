#include "BalancePointLearner.h"

#include <math.h>

static int32_t absInt16(int16_t value) {
  int32_t widened = value;
  return widened < 0 ? -widened : widened;
}

static float clampAlpha(float alpha) {
  if (alpha < 0.0f) {
    return 0.0f;
  }
  if (alpha > 1.0f) {
    return 1.0f;
  }
  return alpha;
}

BalancePointLearner::BalancePointLearner()
    : settleMillis_(0),
      stableMillis_(0),
      minWriteIntervalMillis_(0),
      maxAngleErrorDegrees_(0.0f),
      maxRateDegPerSec_(0.0f),
      maxAbsMotorCommand_(0),
      smoothingAlpha_(0.0f),
      storedBalancePointDegrees_(0.0f),
      resetMillis_(0),
      stableStartMillis_(0),
      lastWriteMillis_(0),
      hasStableStart_(false),
      hasSaved_(false) {}

void BalancePointLearner::configure(unsigned long settleMillis,
                                    unsigned long stableMillis,
                                    unsigned long minWriteIntervalMillis,
                                    float maxAngleErrorDegrees,
                                    float maxRateDegPerSec,
                                    int16_t maxAbsMotorCommand,
                                    float smoothingAlpha) {
  settleMillis_ = settleMillis;
  stableMillis_ = stableMillis;
  minWriteIntervalMillis_ = minWriteIntervalMillis;
  maxAngleErrorDegrees_ = fabs(maxAngleErrorDegrees);
  maxRateDegPerSec_ = fabs(maxRateDegPerSec);
  maxAbsMotorCommand_ = absInt16(maxAbsMotorCommand);
  smoothingAlpha_ = clampAlpha(smoothingAlpha);
}

void BalancePointLearner::reset(float storedBalancePointDegrees,
                                unsigned long nowMillis) {
  storedBalancePointDegrees_ = storedBalancePointDegrees;
  resetMillis_ = nowMillis;
  stableStartMillis_ = 0;
  lastWriteMillis_ = 0;
  hasStableStart_ = false;
  hasSaved_ = false;
}

BalanceLearningResult
BalancePointLearner::update(const SensorFrame& frame,
                            float activeBalancePointDegrees,
                            int16_t balanceOutput,
                            unsigned long nowMillis) {
  BalanceLearningResult result;

  if (nowMillis - resetMillis_ < settleMillis_) {
    hasStableStart_ = false;
    return result;
  }

  if (!isStable(frame, activeBalancePointDegrees, balanceOutput)) {
    hasStableStart_ = false;
    return result;
  }

  if (!hasStableStart_) {
    stableStartMillis_ = nowMillis;
    hasStableStart_ = true;
    return result;
  }

  if (nowMillis - stableStartMillis_ < stableMillis_) {
    return result;
  }

  if (hasSaved_ && nowMillis - lastWriteMillis_ < minWriteIntervalMillis_) {
    return result;
  }

  storedBalancePointDegrees_ =
      storedBalancePointDegrees_ +
      smoothingAlpha_ * (activeBalancePointDegrees - storedBalancePointDegrees_);
  lastWriteMillis_ = nowMillis;
  hasSaved_ = true;

  result.shouldSave = true;
  result.balancePointDegrees = storedBalancePointDegrees_;
  return result;
}

bool BalancePointLearner::isStable(const SensorFrame& frame,
                                   float activeBalancePointDegrees,
                                   int16_t balanceOutput) const {
  if (!frame.gyroFresh) {
    return false;
  }

  return fabs(frame.angleDegrees - activeBalancePointDegrees) <=
             maxAngleErrorDegrees_ &&
         fabs(frame.angleRateDegPerSec) <= maxRateDegPerSec_ &&
         absInt16(balanceOutput) <= maxAbsMotorCommand_;
}
