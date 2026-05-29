#include <cassert>
#include <cstdlib>
#include <iostream>

#include "../../SelfBalanceRobot/Sensors.h"

float MeGyro::angleX = 12.5f;
float MeGyro::angleY = -33.0f;
float MeGyro::angleZ = 4.0f;
float MeGyro::gyroX = 0.0f;
float MeGyro::gyroY = 5.0f;
float MeGyro::gyroZ = 0.0f;

static void test_balance_axis_uses_configured_x_axis() {
  static_assert(Config::BalanceGyroAxis == GyroAxis::X,
                "Default balance axis should match the robot pitch axis");

  Sensors sensors;
  sensors.begin();

  const SensorFrame& frame = sensors.update(100);

  assert(frame.gyroFresh);
  assert(frame.angleDegrees == 12.5f);
}

static void test_repeated_still_angle_remains_fresh() {
  Sensors sensors;
  sensors.begin();

  const SensorFrame& first = sensors.update(100);
  assert(first.gyroFresh);
  assert(first.angleDegrees == 12.5f);

  const SensorFrame& second = sensors.update(110);
  assert(second.gyroFresh);
  assert(second.angleDegrees == 12.5f);
  assert(second.angleRateDegPerSec == 5.0f);
}

int main() {
  test_balance_axis_uses_configured_x_axis();
  test_repeated_still_angle_remains_fresh();

  std::cout << "test_sensors PASS\n";
  return EXIT_SUCCESS;
}
