#ifndef BALANCE_POINT_LEARNER_H
#define BALANCE_POINT_LEARNER_H

#include "RobotTypes.h"

struct BalanceLearningResult {
  bool shouldSave = false;
  float balancePointDegrees = 0.0f;
};

class BalancePointLearner {
public:
  BalancePointLearner();

  void configure(unsigned long settleMillis, unsigned long stableMillis,
                 unsigned long minWriteIntervalMillis,
                 float maxAngleErrorDegrees, float maxRateDegPerSec,
                 int16_t maxAbsMotorCommand, float smoothingAlpha);
  void reset(float storedBalancePointDegrees, unsigned long nowMillis);

  BalanceLearningResult update(const SensorFrame& frame,
                               float activeBalancePointDegrees,
                               int16_t balanceOutput,
                               unsigned long nowMillis);

private:
  bool isStable(const SensorFrame& frame, float activeBalancePointDegrees,
                int16_t balanceOutput) const;

  unsigned long settleMillis_;
  unsigned long stableMillis_;
  unsigned long minWriteIntervalMillis_;
  float maxAngleErrorDegrees_;
  float maxRateDegPerSec_;
  int32_t maxAbsMotorCommand_;
  float smoothingAlpha_;
  float storedBalancePointDegrees_;
  unsigned long resetMillis_;
  unsigned long stableStartMillis_;
  unsigned long lastWriteMillis_;
  bool hasStableStart_;
  bool hasSaved_;
};

#endif
