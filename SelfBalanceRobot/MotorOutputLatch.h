#ifndef MOTOR_OUTPUT_LATCH_H
#define MOTOR_OUTPUT_LATCH_H

#include "RobotTypes.h"

class MotorOutputLatch {
public:
  bool shouldWrite(const MotorCommand& command);
  bool shouldStop();
  void reset();

private:
  MotorCommand lastCommand_;
  bool hasCommand_ = false;
  bool stopped_ = false;
};

#endif
