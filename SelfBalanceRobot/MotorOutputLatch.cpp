#include "MotorOutputLatch.h"

bool MotorOutputLatch::shouldWrite(const MotorCommand& command) {
  if (!stopped_ && hasCommand_ && lastCommand_.left == command.left &&
      lastCommand_.right == command.right) {
    return false;
  }

  lastCommand_ = command;
  hasCommand_ = true;
  stopped_ = false;
  return true;
}

bool MotorOutputLatch::shouldStop() {
  if (stopped_) {
    return false;
  }

  lastCommand_ = MotorCommand();
  hasCommand_ = true;
  stopped_ = true;
  return true;
}

void MotorOutputLatch::reset() {
  lastCommand_ = MotorCommand();
  hasCommand_ = false;
  stopped_ = false;
}
