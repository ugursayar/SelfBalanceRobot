#include "Motors.h"

namespace {
Motors* activeMotors = nullptr;
}

Motors::Motors()
    : rightMotor_(Config::RightEncoderSlot),
      leftMotor_(Config::LeftEncoderSlot), feedback_() {}

void Motors::begin() {
  activeMotors = this;
  attachInterrupt(rightMotor_.getIntNum(), rightEncoderIsr, RISING);
  attachInterrupt(leftMotor_.getIntNum(), leftEncoderIsr, RISING);
  resetTravel();
  stop();
}

WheelFeedback Motors::updateFeedback() {
  rightMotor_.updateSpeed();
  leftMotor_.updateSpeed();

  feedback_.rightPositionDegrees =
      applyPositionInversion(rightMotor_.getCurPos(), Config::InvertRightMotor);
  feedback_.leftPositionDegrees =
      applyPositionInversion(leftMotor_.getCurPos(), Config::InvertLeftMotor);
  feedback_.rightSpeedRpm =
      applySpeedInversion(rightMotor_.getCurrentSpeed(), Config::InvertRightMotor);
  feedback_.leftSpeedRpm =
      applySpeedInversion(leftMotor_.getCurrentSpeed(), Config::InvertLeftMotor);
  feedback_.averagePositionDegrees =
      (static_cast<float>(feedback_.rightPositionDegrees) +
       static_cast<float>(feedback_.leftPositionDegrees)) *
      0.5f;
  feedback_.averageSpeedRpm =
      (feedback_.rightSpeedRpm + feedback_.leftSpeedRpm) * 0.5f;
  return feedback_;
}

const WheelFeedback& Motors::feedback() const { return feedback_; }

void Motors::resetTravel() {
  rightMotor_.setPulsePos(0);
  leftMotor_.setPulsePos(0);
  feedback_ = WheelFeedback();
}

void Motors::write(const MotorCommand& command) {
  rightMotor_.setMotorPwm(applyInversion(command.right, Config::InvertRightMotor));
  leftMotor_.setMotorPwm(applyInversion(command.left, Config::InvertLeftMotor));
}

void Motors::stop() {
  rightMotor_.setMotorPwm(0);
  leftMotor_.setMotorPwm(0);
}

int16_t Motors::applyInversion(int16_t value, bool invert) const {
  return invert ? static_cast<int16_t>(-value) : value;
}

long Motors::applyPositionInversion(long value, bool invert) const {
  return invert ? -value : value;
}

float Motors::applySpeedInversion(float value, bool invert) const {
  return invert ? -value : value;
}

void Motors::handleRightPulse() {
  if (digitalRead(rightMotor_.getPortB()) == 0) {
    rightMotor_.pulsePosMinus();
  } else {
    rightMotor_.pulsePosPlus();
  }
}

void Motors::handleLeftPulse() {
  if (digitalRead(leftMotor_.getPortB()) == 0) {
    leftMotor_.pulsePosMinus();
  } else {
    leftMotor_.pulsePosPlus();
  }
}

void Motors::rightEncoderIsr() {
  if (activeMotors != nullptr) {
    activeMotors->handleRightPulse();
  }
}

void Motors::leftEncoderIsr() {
  if (activeMotors != nullptr) {
    activeMotors->handleLeftPulse();
  }
}
