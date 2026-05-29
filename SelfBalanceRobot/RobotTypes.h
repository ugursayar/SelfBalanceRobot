#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <Arduino.h>

enum class RobotMode : uint8_t {
  Disarmed,
  Calibrating,
  Balancing,
  Fault
};

struct SensorFrame {
  float angleDegrees = 0.0f;
  // Raw gyro rate for the balance axis (deg/s). MeGyro integrates gyrY into
  // getAngleX, so getGyroY() is the correct rate source for BalanceGyroAxis::X.
  float angleRateDegPerSec = 0.0f;
  bool gyroFresh = false;
  unsigned long nowMillis = 0;
};

struct ControlCommand {
  bool arm = false;
  bool stop = false;
  unsigned long receivedMillis = 0;
};

struct MotorCommand {
  int16_t left = 0;
  int16_t right = 0;
};

struct WheelFeedback {
  long leftPositionDegrees = 0;
  long rightPositionDegrees = 0;
  float leftSpeedRpm = 0.0f;
  float rightSpeedRpm = 0.0f;
  float averagePositionDegrees = 0.0f;
  float averageSpeedRpm = 0.0f;
  int16_t leftPwm = 0;
  int16_t rightPwm = 0;
};

#endif
