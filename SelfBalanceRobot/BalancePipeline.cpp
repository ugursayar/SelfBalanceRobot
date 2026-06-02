#include "BalancePipeline.h"

BalancePipelineOutput
BalancePipeline::update(const BalancePipelineInput& input,
                        BalanceController& controller) const {
  BalancePipelineOutput output;

  const float speedCorrection = clampWheelSpeedTargetCorrection(
      input.wheelFeedback.averageSpeedRpm *
      Config::WheelSpeedTargetCorrectionDegreesPerRpm);
  output.travelHoldTargetCorrectionDegrees =
      clampTravelHoldTargetCorrection(
          input.wheelFeedback.averagePositionDegrees *
          Config::TravelHoldTargetDegreesPerWheelDegree);
  output.baseTargetDegrees = baseBalanceTargetDegrees(input);
  const float finalTarget = output.baseTargetDegrees + speedCorrection +
                            output.travelHoldTargetCorrectionDegrees;
  output.targetAngleDegrees = rampedTargetDegrees(input, finalTarget);

  controller.setTargetAngle(output.targetAngleDegrees);
  output.rawBalanceOutput =
      controller.update(input.frame.angleDegrees,
                        input.frame.angleRateDegPerSec, input.dtSeconds);

  const float angleError =
      output.targetAngleDegrees - input.frame.angleDegrees;
  int16_t shapedOutput =
      applyMinimumBalanceCommand(output.rawBalanceOutput, angleError);
  shapedOutput = applyLargeLeanBoost(shapedOutput, angleError);
  shapedOutput =
      applyWheelSpeedDamping(shapedOutput, angleError, input.wheelFeedback);

  output.balanceOutput = shapedOutput;
  output.motorCommand.left = shapedOutput;
  output.motorCommand.right = shapedOutput;
  return output;
}

float BalancePipeline::baseBalanceTargetDegrees(
    const BalancePipelineInput& input) const {
  if (input.balanceSessionUsesPersistedPoint) {
    return input.activeBalancePointDegrees;
  }
  return input.uprightAngleDegrees + input.currentTrimDegrees;
}

float BalancePipeline::rampedTargetDegrees(
    const BalancePipelineInput& input, float finalTargetDegrees) const {
  const unsigned long rampMs = Config::BalanceTargetRampMillis;
  const unsigned long elapsed =
      input.frame.nowMillis - input.balancingStartMillis;
  const float rampStartTarget = input.balanceSessionUsesPersistedPoint
                                    ? baseBalanceTargetDegrees(input)
                                    : input.uprightAngleDegrees;
  if (elapsed < rampMs) {
    const float rampFraction =
        static_cast<float>(elapsed) / static_cast<float>(rampMs);
    return rampStartTarget +
           rampFraction * (finalTargetDegrees - rampStartTarget);
  }
  return finalTargetDegrees;
}

float BalancePipeline::clampWheelSpeedTargetCorrection(
    float correctionDegrees) const {
  const float limit = Config::MaxWheelSpeedTargetCorrectionDegrees;
  if (correctionDegrees > limit) {
    return limit;
  }
  if (correctionDegrees < -limit) {
    return -limit;
  }
  return correctionDegrees;
}

float BalancePipeline::clampTravelHoldTargetCorrection(
    float correctionDegrees) const {
  const float limit = Config::MaxTravelHoldTargetCorrectionDegrees;
  if (correctionDegrees > limit) {
    return limit;
  }
  if (correctionDegrees < -limit) {
    return -limit;
  }
  return correctionDegrees;
}

int16_t BalancePipeline::clampMotorCommand(float command) const {
  if (command > Config::MaxMotorCommand) {
    return Config::MaxMotorCommand;
  }
  if (command < -Config::MaxMotorCommand) {
    return -Config::MaxMotorCommand;
  }
  return static_cast<int16_t>(command);
}

int16_t BalancePipeline::applyLargeLeanBoost(int16_t balanceOutput,
                                             float angleError) const {
  const float absAngleError = angleError < 0.0f ? -angleError : angleError;
  if (absAngleError <= Config::LargeLeanBoostAngleDegrees) {
    return balanceOutput;
  }

  const float extra =
      (absAngleError - Config::LargeLeanBoostAngleDegrees) *
      Config::LargeLeanBoostCommandPerDegree;
  const float correctionDirection = angleError < 0.0f ? 1.0f : -1.0f;
  return clampMotorCommand(static_cast<float>(balanceOutput) +
                           (correctionDirection * extra));
}

int16_t BalancePipeline::applyMinimumBalanceCommand(
    int16_t balanceOutput, float angleError) const {
  if (angleError < 0.0f) {
    angleError = -angleError;
  }
  if (angleError < Config::MinBalanceBoostAngleDegrees) {
    return balanceOutput;
  }

  const int16_t minimum = Config::MinBalanceMotorCommand;
  if (minimum <= 0) {
    return balanceOutput;
  }

  if (balanceOutput > 0 && balanceOutput < minimum) {
    return minimum;
  }
  if (balanceOutput < 0 && balanceOutput > -minimum) {
    return static_cast<int16_t>(-minimum);
  }
  return balanceOutput;
}

int16_t BalancePipeline::applyWheelSpeedDamping(
    int16_t balanceOutput, float angleError,
    const WheelFeedback& wheelFeedback) const {
  const float absAngleError = angleError < 0.0f ? -angleError : angleError;
  if (absAngleError > Config::WheelSpeedDampingMaxAngleErrorDegrees) {
    return balanceOutput;
  }

  return clampMotorCommand(
      static_cast<float>(balanceOutput) -
      (wheelFeedback.averageSpeedRpm *
       Config::WheelSpeedDampingCommandPerRpm));
}
