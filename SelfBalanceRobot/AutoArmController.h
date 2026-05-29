#ifndef AUTO_ARM_CONTROLLER_H
#define AUTO_ARM_CONTROLLER_H

#include "RobotTypes.h"

class AutoArmController {
public:
  AutoArmController();

  void configure(float angleWindowDegrees, float maxRateDegPerSec,
                 unsigned long stillMillis);
  void setTargetBalancePoint(float balancePointDegrees);
  void reset();
  void suppressUntil(unsigned long nowMillis, unsigned long cooldownMillis);
  bool update(const SensorFrame& frame);

private:
  bool updateSuppression(unsigned long nowMillis);

  float angleWindowDegrees_;
  float maxRateDegPerSec_;
  unsigned long stillMillis_;
  float targetBalancePointDegrees_;
  unsigned long candidateStartMillis_;
  unsigned long suppressedUntilMillis_;
  bool hasCandidate_;
  bool suppressionActive_;
};

#endif
