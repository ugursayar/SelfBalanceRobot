#include "RobotState.h"

#include <math.h>

RobotState::RobotState()
    : mode_(RobotMode::Disarmed), fallAngleDegrees_(30.0f),
      obstacleDistanceCm_(20.0f), calibrationMillis_(1000),
      commandTimeoutMillis_(250), calibrationStartMillis_(0),
      calibrationSumDegrees_(0.0f), calibrationSamples_(0),
      uprightAngleDegrees_(0.0f) {}

void RobotState::configure(float fallAngleDegrees, float obstacleDistanceCm,
                           unsigned long calibrationMillis,
                           unsigned long commandTimeoutMillis) {
  fallAngleDegrees_ = fallAngleDegrees;
  obstacleDistanceCm_ = obstacleDistanceCm;
  calibrationMillis_ = calibrationMillis;
  commandTimeoutMillis_ = commandTimeoutMillis;
}

void RobotState::update(const SensorFrame& frame,
                        const ControlCommand& command) {
  if (command.stop) {
    mode_ = RobotMode::Disarmed;
    return;
  }

  switch (mode_) {
  case RobotMode::Disarmed:
    if (command.arm && frame.gyroFresh &&
        commandIsFresh(command, frame.nowMillis)) {
      startCalibration(frame);
      if (frame.nowMillis - calibrationStartMillis_ >= calibrationMillis_) {
        finishCalibration();
      }
    }
    break;

  case RobotMode::Calibrating:
    if (frame.nowMillis - calibrationStartMillis_ >= calibrationMillis_) {
      if (!frame.gyroFresh) {
        mode_ = RobotMode::Fault;
        break;
      }
      addCalibrationSample(frame);
      finishCalibration();
    } else {
      addCalibrationSample(frame);
    }
    break;

  case RobotMode::Balancing:
    if (!frame.gyroFresh || hasFallen(frame.angleDegrees)) {
      mode_ = RobotMode::Fault;
    } else if (command.driveEnabled &&
               commandIsFresh(command, frame.nowMillis)) {
      mode_ = RobotMode::Drive;
    }
    break;

  case RobotMode::Drive:
    if (!frame.gyroFresh || hasFallen(frame.angleDegrees)) {
      mode_ = RobotMode::Fault;
    } else if (!command.driveEnabled ||
               !commandIsFresh(command, frame.nowMillis)) {
      mode_ = RobotMode::Balancing;
    }
    break;

  case RobotMode::Fault:
    break;
  }
}

ControlCommand RobotState::safeCommand(const ControlCommand& command,
                                       const SensorFrame& frame) const {
  ControlCommand safe = command;

  if (mode_ != RobotMode::Drive ||
      !commandIsFresh(command, frame.nowMillis)) {
    safe.forward = 0;
    safe.turn = 0;
    return safe;
  }

  if (frame.ultrasonicFresh && frame.distanceCm > 0.0f &&
      frame.distanceCm < obstacleDistanceCm_ && safe.forward > 0) {
    safe.forward = 0;
  }

  return safe;
}

RobotMode RobotState::mode() const { return mode_; }

bool RobotState::motorsEnabled() const {
  return mode_ == RobotMode::Balancing || mode_ == RobotMode::Drive;
}

float RobotState::uprightAngleDegrees() const { return uprightAngleDegrees_; }

bool RobotState::commandIsFresh(const ControlCommand& command,
                                unsigned long nowMillis) const {
  return nowMillis - command.receivedMillis <= commandTimeoutMillis_;
}

bool RobotState::hasFallen(float angleDegrees) const {
  return fabs(angleDegrees - uprightAngleDegrees_) >= fallAngleDegrees_;
}

void RobotState::startCalibration(const SensorFrame& frame) {
  mode_ = RobotMode::Calibrating;
  calibrationStartMillis_ = frame.nowMillis;
  calibrationSumDegrees_ = 0.0f;
  calibrationSamples_ = 0;
  addCalibrationSample(frame);
}

void RobotState::addCalibrationSample(const SensorFrame& frame) {
  if (frame.gyroFresh) {
    calibrationSumDegrees_ += frame.angleDegrees;
    ++calibrationSamples_;
  }
}

void RobotState::finishCalibration() {
  if (calibrationSamples_ > 0) {
    uprightAngleDegrees_ = calibrationSumDegrees_ / calibrationSamples_;
    mode_ = RobotMode::Balancing;
  } else {
    mode_ = RobotMode::Fault;
  }
}
