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

private:
  MeEncoderOnBoard rightMotor_;
  MeEncoderOnBoard leftMotor_;
  WheelFeedback feedback_;

  int16_t applyInversion(int16_t value, bool invert) const;
  long applyPositionInversion(long value, bool invert) const;
  float applySpeedInversion(float value, bool invert) const;
  void handleRightPulse();
  void handleLeftPulse();

  static void rightEncoderIsr();
  static void leftEncoderIsr();
};

#endif
