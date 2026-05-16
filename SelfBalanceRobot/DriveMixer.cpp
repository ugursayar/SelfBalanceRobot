#include "DriveMixer.h"

namespace {
int16_t positiveLimit(int16_t limit) {
  const int32_t widenedLimit = limit;
  const int32_t positive = widenedLimit < 0 ? -widenedLimit : widenedLimit;
  return positive > INT16_MAX ? INT16_MAX : static_cast<int16_t>(positive);
}
} // namespace

DriveMixer::DriveMixer()
    : maxMotorCommand_(255), maxDriveCommand_(80), maxTurnCommand_(80),
      deadband_(0) {}

void DriveMixer::setLimits(int16_t maxMotorCommand, int16_t maxDriveCommand,
                           int16_t maxTurnCommand, int16_t deadband) {
  maxMotorCommand_ = positiveLimit(maxMotorCommand);
  maxDriveCommand_ = positiveLimit(maxDriveCommand);
  maxTurnCommand_ = positiveLimit(maxTurnCommand);
  deadband_ = positiveLimit(deadband);
}

MotorCommand DriveMixer::mix(int16_t balanceOutput, int16_t forwardCommand,
                             int16_t turnCommand) const {
  const int16_t drive = clampValue(forwardCommand, maxDriveCommand_);
  const int16_t turn = clampValue(turnCommand, maxTurnCommand_);

  MotorCommand command;
  command.left = applyDeadband(
      clampValue(static_cast<int32_t>(balanceOutput) + drive - turn,
                 maxMotorCommand_));
  command.right = applyDeadband(
      clampValue(static_cast<int32_t>(balanceOutput) + drive + turn,
                 maxMotorCommand_));
  return command;
}

int16_t DriveMixer::clampValue(int32_t value, int16_t limit) const {
  const int32_t positiveLimitValue = positiveLimit(limit);
  if (value > positiveLimitValue) {
    return static_cast<int16_t>(positiveLimitValue);
  }
  if (value < -positiveLimitValue) {
    return static_cast<int16_t>(-positiveLimitValue);
  }
  return static_cast<int16_t>(value);
}

int16_t DriveMixer::applyDeadband(int16_t value) const {
  if (value >= -deadband_ && value <= deadband_) {
    return 0;
  }
  return value;
}
