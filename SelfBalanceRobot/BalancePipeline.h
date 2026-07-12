#ifndef BALANCE_PIPELINE_H
#define BALANCE_PIPELINE_H

#include "BalanceController.h"
#include "LqrController.h"
#include "RobotTypes.h"
#include "config.h"

struct BalancePipelineInput {
  SensorFrame frame;
  WheelFeedback wheelFeedback;
  float uprightAngleDegrees = 0.0f;
  float activeBalancePointDegrees = Config::AutoArmDefaultBalancePointDegrees;
  float currentTrimDegrees = Config::BalanceAngleTrimDegrees;
  unsigned long balancingStartMillis = 0;
  bool balanceSessionUsesPersistedPoint = false;
  float dtSeconds = 0.0f;
};

struct BalancePipelineOutput {
  float baseTargetDegrees = 0.0f;
  float targetAngleDegrees = 0.0f;
  float travelHoldTargetCorrectionDegrees = 0.0f;
  int16_t rawBalanceOutput = 0;
  int16_t balanceOutput = 0;
  MotorCommand motorCommand;
};

class BalancePipeline {
public:
  // Runs the active controller (LQR when Config::EnableLqrController, otherwise
  // the PID BalanceController) and applies the shared output shaping.  Both
  // controllers are passed in; the inactive one is left untouched.
  BalancePipelineOutput update(const BalancePipelineInput& input,
                               BalanceController& controller,
                               LqrController& lqr) const;

private:
  float baseBalanceTargetDegrees(const BalancePipelineInput& input) const;
  float rampedTargetDegrees(const BalancePipelineInput& input,
                            float baseTargetDegrees,
                            float finalTargetDegrees) const;
  static float clampSymmetric(float value, float limit);
  float clampWheelSpeedTargetCorrection(float correctionDegrees) const;
  float clampTravelHoldTargetCorrection(float correctionDegrees) const;
  int16_t clampMotorCommand(float command) const;
  int16_t applyLargeLeanBoost(int16_t balanceOutput, float angleError) const;
  int16_t applyMinimumBalanceCommand(int16_t balanceOutput,
                                     float angleError) const;
  int16_t applyWheelSpeedDamping(int16_t balanceOutput, float angleError,
                                 const WheelFeedback& wheelFeedback) const;
};

#endif
