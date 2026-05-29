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

int main() {
  test_balance_axis_uses_configured_x_axis();

  std::cout << "test_sensors PASS\n";
  return EXIT_SUCCESS;
}
