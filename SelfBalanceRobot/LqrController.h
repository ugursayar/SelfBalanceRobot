#ifndef LQR_CONTROLLER_H
#define LQR_CONTROLLER_H

#include <stdint.h>

// Linear-Quadratic Regulator state-feedback controller.
//
// Applies a fixed full-state gain row K each tick: u = K * (x - x*), where the
// state is x = [angle, angleRate, wheelPosition, wheelSpeed].  K is computed
// offline (solve the Riccati equation for the linearized plant + chosen Q/R);
// this class just evaluates the row, clamps to the output limit, and exposes
// the same telemetry getters as BalanceController so it drops into the balance
// pipeline interchangeably.
//
// Unlike BalanceController this law is purely linear: there is no proportional
// gain schedule and no integral term.  Gains are expressed in the output
// (motor-command) sign convention, so a positive gain yields a corrective
// command in the same direction as the PID kp/kd terms.
class LqrController {
public:
  LqrController();

  void setGains(float angleGain, float angleRateGain, float wheelPositionGain,
                float wheelSpeedGain);
  // Scales the angle (proportional) gain DOWN for small tilt deviations: at the
  // target the effective angle gain is (smallErrorGainScale * angleGain),
  // ramping linearly up to the full angle gain at smallErrorDegrees and beyond.
  // Softens the motor response to small corrections near upright without
  // weakening big-lean recovery.  Mirrors BalanceController's schedule; pass a
  // scale of 1.0 (the default) to keep the law purely linear.
  void setGainSchedule(float smallErrorDegrees, float smallErrorGainScale);
  void setRateFilter(float filterAlpha);
  void setTargetAngle(float targetAngleDegrees);
  void setOutputLimit(int16_t outputLimit);
  void reset();

  // u = K * (x - x*).  wheelPositionDegrees / wheelSpeedRpm are ignored when
  // their gains are zero (the default), reducing the law to angle/rate feedback.
  int16_t update(float measuredAngleDegrees, float measuredAngleRateDegPerSec,
                 float wheelPositionDegrees, float wheelSpeedRpm,
                 float dtSeconds);

  float lastErrorDegrees() const;
  float lastMeasuredAngleRateDegreesPerSecond() const;

private:
  float scheduledAngleGain(float angleDeviation) const;

  float kAngle_;
  float kAngleRate_;
  float kWheelPosition_;
  float kWheelSpeed_;
  float smallErrorDegrees_;
  float smallErrorGainScale_;
  float rateFilterAlpha_;
  float filteredMeasuredAngleRateDegreesPerSecond_;
  float targetAngleDegrees_;
  float lastErrorDegrees_;
  float lastMeasuredAngleRateDegreesPerSecond_;
  int16_t outputLimit_;
};

#endif
