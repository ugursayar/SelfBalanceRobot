#include "Sensors.h"

Sensors::Sensors()
    : gyro_(Config::GyroPort), ultrasonic_(Config::UltrasonicPort), frame_(),
      lastUltrasonicMillis_(0), hasUltrasonicSample_(false) {}

void Sensors::begin() {
  gyro_.begin();
  frame_ = SensorFrame();
  lastUltrasonicMillis_ = 0;
  hasUltrasonicSample_ = false;
}

const SensorFrame& Sensors::update(unsigned long nowMillis) {
  frame_.nowMillis = nowMillis;
  frame_.gyroFresh = false;
  frame_.ultrasonicFresh = hasUltrasonicSample_;

  gyro_.update();
  frame_.angleDegrees = gyro_.angleY();
  frame_.gyroFresh = true;

  if (!hasUltrasonicSample_ ||
      nowMillis - lastUltrasonicMillis_ >= Config::UltrasonicPeriodMillis) {
    frame_.distanceCm = ultrasonic_.distanceCm();
    lastUltrasonicMillis_ = nowMillis;
    hasUltrasonicSample_ = true;
    frame_.ultrasonicFresh = true;
  }

  return frame_;
}

const SensorFrame& Sensors::current() const { return frame_; }
