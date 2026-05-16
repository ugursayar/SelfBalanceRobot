#ifndef DRIVE_MIXER_H
#define DRIVE_MIXER_H

#include "RobotTypes.h"

class DriveMixer {
public:
  DriveMixer();

  void setLimits(int16_t maxMotorCommand, int16_t maxDriveCommand,
                 int16_t maxTurnCommand, int16_t deadband);
  MotorCommand mix(int16_t balanceOutput, int16_t forwardCommand,
                   int16_t turnCommand) const;

private:
  int16_t maxMotorCommand_;
  int16_t maxDriveCommand_;
  int16_t maxTurnCommand_;
  int16_t deadband_;

  int16_t clampValue(int32_t value, int16_t limit) const;
  int16_t applyDeadband(int16_t value) const;
};

#endif
