#include "Motors.h"

Motors::Motors()
    : rightMotor_(Config::RightMotorPort), leftMotor_(Config::LeftMotorPort) {}

void Motors::begin() { stop(); }

void Motors::write(const MotorCommand& command) {
  rightMotor_.run(applyInversion(command.right, Config::InvertRightMotor));
  leftMotor_.run(applyInversion(command.left, Config::InvertLeftMotor));
}

void Motors::stop() {
  rightMotor_.stop();
  leftMotor_.stop();
}

int16_t Motors::applyInversion(int16_t value, bool invert) const {
  return invert ? static_cast<int16_t>(-value) : value;
}
