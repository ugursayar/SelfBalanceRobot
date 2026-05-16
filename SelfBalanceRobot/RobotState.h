#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include "RobotTypes.h"

class RobotState {
public:
  RobotState();

  void configure(float fallAngleDegrees, float stillAngleDeltaDegrees,
                 float obstacleDistanceCm, unsigned long calibrationMillis,
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
  bool calibrationArmIsFresh(unsigned long nowMillis) const;
  bool hasFallen(float angleDegrees) const;
  void startCalibration(const SensorFrame& frame,
                        const ControlCommand& command);
  bool addCalibrationSample(const SensorFrame& frame);
  void finishCalibration(const SensorFrame& frame);

  RobotMode mode_;
  float fallAngleDegrees_;
  float stillAngleDeltaDegrees_;
  float obstacleDistanceCm_;
  unsigned long calibrationMillis_;
  unsigned long commandTimeoutMillis_;
  unsigned long calibrationStartMillis_;
  unsigned long calibrationArmMillis_;
  float calibrationInitialAngleDegrees_;
  float calibrationSumDegrees_;
  uint32_t calibrationSamples_;
  float uprightAngleDegrees_;
};

#endif
