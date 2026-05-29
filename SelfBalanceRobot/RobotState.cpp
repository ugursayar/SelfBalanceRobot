#include "RobotState.h"

#include "config.h"

#include <math.h>

RobotState::RobotState()
    : mode_(RobotMode::Disarmed), fallAngleDegrees_(30.0f),
      stillAngleDeltaDegrees_(4.0f), calibrationMillis_(1000),
      commandTimeoutMillis_(250),
      calibrationStartMillis_(0), calibrationArmMillis_(0),
      calibrationInitialAngleDegrees_(0.0f),
      calibrationMinAngleDegrees_(0.0f), calibrationMaxAngleDegrees_(0.0f),
      calibrationSumDegrees_(0.0f), calibrationSamples_(0),
      uprightAngleDegrees_(0.0f) {}

void RobotState::configure(float fallAngleDegrees,
                           float stillAngleDeltaDegrees,
                           unsigned long calibrationMillis,
                           unsigned long commandTimeoutMillis) {
  fallAngleDegrees_ = fallAngleDegrees;
  stillAngleDeltaDegrees_ = stillAngleDeltaDegrees;
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
      startCalibration(frame, command);
      if (frame.nowMillis - calibrationStartMillis_ >= calibrationMillis_) {
        finishCalibration(frame);
      }
    }
    break;

  case RobotMode::Calibrating:
    if (frame.nowMillis - calibrationStartMillis_ >= calibrationMillis_) {
      if (!frame.gyroFresh) {
        mode_ = RobotMode::Fault;
        break;
      }
      if (addCalibrationSample(frame)) {
        finishCalibration(frame);
      }
    } else if (!addCalibrationSample(frame)) {
      mode_ = RobotMode::Fault;
    }
    break;

  case RobotMode::Balancing:
    if (!frame.gyroFresh || hasFallen(frame.angleDegrees)) {
      mode_ = RobotMode::Fault;
    }
    break;

  case RobotMode::Fault:
    break;
  }
}

bool RobotState::startBalancingAt(float uprightAngleDegrees) {
  if (mode_ != RobotMode::Disarmed) {
    return false;
  }
  if (uprightAngleDegrees != uprightAngleDegrees ||
      fabs(uprightAngleDegrees) > Config::MaxStartupUprightAngleDegrees) {
    return false;
  }

  calibrationInitialAngleDegrees_ = uprightAngleDegrees;
  calibrationMinAngleDegrees_ = uprightAngleDegrees;
  calibrationMaxAngleDegrees_ = uprightAngleDegrees;
  calibrationSumDegrees_ = uprightAngleDegrees;
  calibrationSamples_ = 1;
  uprightAngleDegrees_ = uprightAngleDegrees;
  mode_ = RobotMode::Balancing;
  return true;
}

RobotMode RobotState::mode() const { return mode_; }

bool RobotState::motorsEnabled() const {
  return mode_ == RobotMode::Balancing;
}

float RobotState::uprightAngleDegrees() const { return uprightAngleDegrees_; }

float RobotState::calibrationRangeDegrees() const {
  return calibrationMaxAngleDegrees_ - calibrationMinAngleDegrees_;
}

bool RobotState::commandIsFresh(const ControlCommand& command,
                                unsigned long nowMillis) const {
  return nowMillis - command.receivedMillis <= commandTimeoutMillis_;
}

bool RobotState::calibrationArmIsFresh(unsigned long nowMillis) const {
  return nowMillis - calibrationArmMillis_ <= commandTimeoutMillis_;
}

bool RobotState::hasFallen(float angleDegrees) const {
  return fabs(angleDegrees - uprightAngleDegrees_) >= fallAngleDegrees_;
}

void RobotState::startCalibration(const SensorFrame& frame,
                                  const ControlCommand& command) {
  mode_ = RobotMode::Calibrating;
  calibrationStartMillis_ = frame.nowMillis;
  calibrationArmMillis_ = command.receivedMillis;
  calibrationInitialAngleDegrees_ = frame.angleDegrees;
  calibrationMinAngleDegrees_ = frame.angleDegrees;
  calibrationMaxAngleDegrees_ = frame.angleDegrees;
  calibrationSumDegrees_ = 0.0f;
  calibrationSamples_ = 0;
  addCalibrationSample(frame);
}

bool RobotState::addCalibrationSample(const SensorFrame& frame) {
  if (!frame.gyroFresh) {
    return true;
  }

  if (frame.angleDegrees < calibrationMinAngleDegrees_) {
    calibrationMinAngleDegrees_ = frame.angleDegrees;
  }
  if (frame.angleDegrees > calibrationMaxAngleDegrees_) {
    calibrationMaxAngleDegrees_ = frame.angleDegrees;
  }

  if (fabs(frame.angleDegrees - calibrationInitialAngleDegrees_) >
      stillAngleDeltaDegrees_) {
    return false;
  }

  calibrationSumDegrees_ += frame.angleDegrees;
  ++calibrationSamples_;
  return true;
}

void RobotState::finishCalibration(const SensorFrame& frame) {
  if (calibrationSamples_ == 0 || !calibrationArmIsFresh(frame.nowMillis)) {
    mode_ = RobotMode::Fault;
    return;
  }

  const float uprightAngle = calibrationSumDegrees_ / calibrationSamples_;
  if (fabs(uprightAngle) > Config::MaxStartupUprightAngleDegrees) {
    mode_ = RobotMode::Fault;
    return;
  }

  uprightAngleDegrees_ = uprightAngle;
  mode_ = RobotMode::Balancing;
}
