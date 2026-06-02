#ifndef BALANCE_PIPELINE_H
#define BALANCE_PIPELINE_H

#include "BalanceController.h"
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
  BalancePipelineOutput update(const BalancePipelineInput& input,
                               BalanceController& controller) const;

private:
  float baseBalanceTargetDegrees(const BalancePipelineInput& input) const;
  float rampedTargetDegrees(const BalancePipelineInput& input,
                            float finalTargetDegrees) const;
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
