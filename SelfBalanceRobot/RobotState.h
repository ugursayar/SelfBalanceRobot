#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include "RobotTypes.h"

class RobotState {
public:
  RobotState();

  void configure(float fallAngleDegrees, float obstacleDistanceCm,
                 unsigned long calibrationMillis,
                 unsigned long commandTimeoutMillis);
  void update(const SensorFrame& frame, const ControlCommand& command);
  ControlCommand safeCommand(const ControlCommand& command,
                             const SensorFrame& frame) const;

  RobotMode mode() const;
  bool motorsEnabled() const;
  float uprightAngleDegrees() const;

private:
  bool commandIsFresh(const ControlCommand& command,
                      unsigned long nowMillis) const;
  bool hasFallen(float angleDegrees) const;
  void startCalibration(const SensorFrame& frame);
  void addCalibrationSample(const SensorFrame& frame);
  void finishCalibration();

  RobotMode mode_;
  float fallAngleDegrees_;
  float obstacleDistanceCm_;
  unsigned long calibrationMillis_;
  unsigned long commandTimeoutMillis_;
  unsigned long calibrationStartMillis_;
  float calibrationSumDegrees_;
  uint32_t calibrationSamples_;
  float uprightAngleDegrees_;
};

#endif
