#include "Motors.h"

namespace {
Motors* activeMotors = nullptr;

void configureMegaPiEncoderPwmTimers() {
  // Matches the MakeBlock Me_Megapi_encoder_direct example for slots 1 and 2.
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}
}

Motors::Motors()
    : rightMotor_(Config::RightEncoderSlot),
      leftMotor_(Config::LeftEncoderSlot), feedback_() {}

void Motors::begin() {
  configureMegaPiEncoderPwmTimers();
  activeMotors = this;
  if (Config::EnableMotorFeedback) {
    // CHANGE captures both edges for 2× quadrature resolution.
    attachInterrupt(rightMotor_.getIntNum(), rightEncoderIsr, CHANGE);
    attachInterrupt(leftMotor_.getIntNum(), leftEncoderIsr, CHANGE);
  }
  resetTravel();
  stop();
}

WheelFeedback Motors::updateFeedback(MotorFeedbackMode mode) {
  if (!Config::EnableMotorFeedback) {
    return feedback_;
  }

  if (mode == MotorFeedbackMode::Full) {
    rightMotor_.updateSpeed();
    leftMotor_.updateSpeed();
    feedback_.rightSpeedRpm = -rightMotor_.getCurrentSpeed();
    feedback_.leftSpeedRpm = leftMotor_.getCurrentSpeed();
    feedback_.averageSpeedRpm =
        (feedback_.rightSpeedRpm + feedback_.leftSpeedRpm) * 0.5f;
  }

  rightMotor_.updateCurPos();
  leftMotor_.updateCurPos();

  feedback_.rightPositionDegrees = -rightMotor_.getCurPos();
  feedback_.leftPositionDegrees = leftMotor_.getCurPos();
  feedback_.averagePositionDegrees =
      (static_cast<float>(feedback_.rightPositionDegrees) +
       static_cast<float>(feedback_.leftPositionDegrees)) *
      0.5f;
  feedback_.rightPwm = -rightMotor_.getCurPwm();
  feedback_.leftPwm = leftMotor_.getCurPwm();
  return feedback_;
}

const WheelFeedback& Motors::feedback() const { return feedback_; }

void Motors::resetTravel() {
  rightMotor_.setPulsePos(0);
  leftMotor_.setPulsePos(0);
  feedback_ = WheelFeedback();
}

void Motors::write(const MotorCommand& command) {
  // Net drive direction inverted after the motor reassembly: a positive command
  // must move the base toward a forward lean to recover. Both wheels are flipped
  // together so they stay paired (left/right relative mirror is unchanged).
  rightMotor_.setMotorPwm(command.right);
  leftMotor_.setMotorPwm(static_cast<int16_t>(-command.left));
  refreshPwmFeedback();
}

void Motors::stop() {
  rightMotor_.setMotorPwm(0);
  leftMotor_.setMotorPwm(0);
  refreshPwmFeedback();
}

void Motors::refreshPwmFeedback() {
  feedback_.rightPwm = -rightMotor_.getCurPwm();
  feedback_.leftPwm = leftMotor_.getCurPwm();
}

void Motors::handleRightPulse() {
  // On CHANGE interrupts the direction signal is: portB XOR falling_edge.
  // A falling edge on the A channel means the pulse-train level is now LOW,
  // which inverts the normal (RISING-only) quadrature rule.
  const bool risingEdge = (digitalRead(rightMotor_.getPortA()) != 0);
  const bool portB      = (digitalRead(rightMotor_.getPortB()) != 0);
  if (risingEdge ? portB : !portB) {
    rightMotor_.pulsePosMinus();
  } else {
    rightMotor_.pulsePosPlus();
  }
}

void Motors::handleLeftPulse() {
  const bool risingEdge = (digitalRead(leftMotor_.getPortA()) != 0);
  const bool portB      = (digitalRead(leftMotor_.getPortB()) != 0);
  if (risingEdge ? portB : !portB) {
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
