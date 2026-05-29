#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <stdint.h>

class BalanceController {
public:
  BalanceController();

  void setTunings(float kp, float ki, float kd);
  void setIntegralLimit(float integralLimitDegreesSeconds);
  void setRateFilter(float filterAlpha);
  void setTargetAngle(float targetAngleDegrees);
  void setOutputLimit(int16_t outputLimit);
  void reset();
  // 2-param: computes rate by finite-differencing consecutive angle samples.
  int16_t update(float measuredAngleDegrees, float dtSeconds);
  // 3-param: uses the provided raw gyro rate directly (preferred, lower latency).
  int16_t update(float measuredAngleDegrees, float measuredAngleRateDegPerSec,
                 float dtSeconds);
  float lastErrorDegrees() const;
  float lastMeasuredAngleRateDegreesPerSecond() const;

private:
  float kp_;
  float ki_;
  float kd_;
  float integralLimitDegreesSeconds_;
  float targetAngleDegrees_;
  float integral_;
  float previousMeasuredAngleDegrees_;
  float filteredMeasuredAngleRateDegreesPerSecond_;
  float lastErrorDegrees_;
  float lastMeasuredAngleRateDegreesPerSecond_;
  float rateFilterAlpha_;
  int16_t outputLimit_;
  bool hasPreviousMeasuredAngle_;
};

#endif
