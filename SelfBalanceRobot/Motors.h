#ifndef MOTORS_H
#define MOTORS_H

#include <MeEncoderOnBoard.h>

#include "RobotTypes.h"
#include "config.h"

class Motors {
public:
  Motors();

  void begin();
  WheelFeedback updateFeedback();
  const WheelFeedback& feedback() const;
  void resetTravel();
  void write(const MotorCommand& command);
  void stop();
  void refreshPwmFeedback();

private:
  MeEncoderOnBoard rightMotor_;
  MeEncoderOnBoard leftMotor_;
  WheelFeedback feedback_;

  void handleRightPulse();
  void handleLeftPulse();

  static void rightEncoderIsr();
  static void leftEncoderIsr();
};

#endif
