#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <Arduino.h>

enum class RobotMode : uint8_t {
  Disarmed,
  Calibrating,
  Balancing,
  Drive,
  Fault
};

struct SensorFrame {
  float angleDegrees = 0.0f;
  float distanceCm = 400.0f;
  bool gyroFresh = false;
  bool ultrasonicFresh = false;
  unsigned long nowMillis = 0;
};

struct ControlCommand {
  bool arm = false;
  bool stop = false;
  bool driveEnabled = false;
  int16_t forward = 0;
  int16_t turn = 0;
  float tuneKp = 0.0f;
  float tuneKi = 0.0f;
  float tuneKd = 0.0f;
  float tuneTrimDegrees = 0.0f;
  bool hasTuning = false;
  bool hasTrim = false;
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
};

#endif
