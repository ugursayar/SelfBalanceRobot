#include "SafetyCutoff.h"

namespace {
float absFloat(float value) {
  return value < 0.0f ? -value : value;
}

int16_t absInt16(int16_t value) {
  return value < 0 ? static_cast<int16_t>(-value) : value;
}
}

SafetyCutoff::SafetyCutoff()
    : angleErrorDegrees_(0.0f), motorCommand_(0), cutoffMillis_(0),
      unsafeStartMillis_(0), unsafeActive_(false) {}

void SafetyCutoff::configure(float angleErrorDegrees, int16_t motorCommand,
                             unsigned long cutoffMillis) {
  angleErrorDegrees_ = absFloat(angleErrorDegrees);
  motorCommand_ = absInt16(motorCommand);
  cutoffMillis_ = cutoffMillis;
  reset();
}

void SafetyCutoff::reset() {
  unsafeStartMillis_ = 0;
  unsafeActive_ = false;
}

bool SafetyCutoff::update(float angleErrorDegrees, int16_t motorCommand,
                          unsigned long nowMillis) {
  if (cutoffMillis_ == 0 || !unsafe(angleErrorDegrees, motorCommand)) {
    reset();
    return false;
  }

  if (!unsafeActive_) {
    unsafeStartMillis_ = nowMillis;
    unsafeActive_ = true;
    return false;
  }

  return nowMillis - unsafeStartMillis_ >= cutoffMillis_;
}

bool SafetyCutoff::unsafe(float angleErrorDegrees,
                          int16_t motorCommand) const {
  return absFloat(angleErrorDegrees) >= angleErrorDegrees_ &&
         absInt16(motorCommand) >= motorCommand_;
}
