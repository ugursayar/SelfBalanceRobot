#include "Sensors.h"

Sensors::Sensors()
    : gyro_(Config::GyroPort), frame_(), lastAngleDegrees_(0.0f),
      hasLastAngle_(false) {}

void Sensors::begin() {
  gyro_.begin();
  frame_ = SensorFrame();
  lastAngleDegrees_ = 0.0f;
  hasLastAngle_ = false;
}

const SensorFrame& Sensors::update(unsigned long nowMillis) {
  frame_.nowMillis = nowMillis;
  frame_.gyroFresh = false;

  gyro_.update();
  const float newAngle = readBalanceAngle();
  // MeGyro integrates gyrY into getAngleX (axis swap in update()), so
  // getGyroY() is the zero-latency rate for BalanceGyroAxis::X.
  frame_.angleRateDegPerSec = static_cast<float>(gyro_.getGyroY());

  frame_.angleDegrees = newAngle;
  lastAngleDegrees_ = newAngle;
  hasLastAngle_ = true;
  frame_.gyroFresh = true;

  return frame_;
}

const SensorFrame& Sensors::current() const { return frame_; }

float Sensors::readBalanceAngle() const {
  switch (Config::BalanceGyroAxis) {
  case GyroAxis::X:
    return gyro_.getAngleX();
  case GyroAxis::Y:
    return gyro_.getAngleY();
  case GyroAxis::Z:
    return gyro_.getAngleZ();
  }

  return gyro_.getAngleX();
}
