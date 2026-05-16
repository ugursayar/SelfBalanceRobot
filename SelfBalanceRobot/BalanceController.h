#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <stdint.h>

class BalanceController {
public:
  BalanceController();

  void setTunings(float kp, float ki, float kd);
  void setTargetAngle(float targetAngleDegrees);
  void setOutputLimit(int16_t outputLimit);
  void reset();
  int16_t update(float measuredAngleDegrees, float dtSeconds);

private:
  float kp_;
  float ki_;
  float kd_;
  float targetAngleDegrees_;
  float integral_;
  float previousError_;
  int16_t outputLimit_;
  bool hasPreviousError_;
};

#endif
