#ifndef MOTORS_H
#define MOTORS_H

#include <MeMegaPiDCMotor.h>

#include "RobotTypes.h"
#include "config.h"

class Motors {
public:
  Motors();

  void begin();
  void write(const MotorCommand& command);
  void stop();

private:
  MeMegaPiDCMotor rightMotor_;
  MeMegaPiDCMotor leftMotor_;

  int16_t applyInversion(int16_t value, bool invert) const;
};

#endif
