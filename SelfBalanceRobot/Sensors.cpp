#include "Sensors.h"

namespace {
float clampRate(float rate) {
  const float kMax = 150.0f;
  if (rate > kMax) {
    return kMax;
  }
  if (rate < -kMax) {
    return -kMax;
  }
  return rate;
}
}  // namespace

Sensors::Sensors() : gyro_(Config::GyroPort), frame_() {}

void Sensors::begin() {
  gyro_.begin();
  frame_ = SensorFrame();
}

const SensorFrame& Sensors::update(unsigned long nowMillis) {
  frame_.nowMillis = nowMillis;
  frame_.gyroFresh = false;

  gyro_.update();
  const float newAngle = readBalanceAngle();

  // Only a no-lag clamp on absurd vibration spikes; the kd damping term needs a
  // fast rate signal, so no smoothing here beyond the controller's rate filter.
  frame_.angleRateDegPerSec = clampRate(readBalanceAngleRate());

  frame_.angleDegrees = newAngle;
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

float Sensors::readBalanceAngleRate() const {
  float rawRate = 0.0f;
  switch (Config::BalanceGyroAxis) {
  case GyroAxis::X:
    // MeGyro integrates gyrY into getAngleX, so use gyrY for X pitch.
    rawRate = static_cast<float>(gyro_.getGyroY());
    break;
  case GyroAxis::Y:
    rawRate = static_cast<float>(gyro_.getGyroX());
    break;
  case GyroAxis::Z:
    rawRate = static_cast<float>(gyro_.getGyroZ());
    break;
  }

  return rawRate * Config::BalanceGyroRateSign;
}
