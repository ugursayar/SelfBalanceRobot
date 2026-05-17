#include <cassert>
#include <cstdlib>
#include <iostream>

#include "../../SelfBalanceRobot/DriftController.h"

static WheelFeedback feedback(float positionDegrees, float speedRpm) {
  WheelFeedback value;
  value.averagePositionDegrees = positionDegrees;
  value.averageSpeedRpm = speedRpm;
  return value;
}

static void test_position_and_speed_push_target_against_travel() {
  DriftController controller;
  controller.configure(0.01f, 0.1f, 5.0f, false);
  controller.reset(feedback(100.0f, 0.0f));

  const float correction = controller.update(feedback(150.0f, 10.0f));

  assert(correction < -1.49f);
  assert(correction > -1.51f);
}

static void test_correction_is_limited() {
  DriftController controller;
  controller.configure(1.0f, 1.0f, 2.0f, false);
  controller.reset(feedback(0.0f, 0.0f));

  assert(controller.update(feedback(100.0f, 100.0f)) == -2.0f);
  assert(controller.update(feedback(-100.0f, -100.0f)) == 2.0f);
}

static void test_inversion_flips_direction() {
  DriftController controller;
  controller.configure(0.01f, 0.0f, 5.0f, true);
  controller.reset(feedback(0.0f, 0.0f));

  assert(controller.update(feedback(100.0f, 0.0f)) == 1.0f);
}

int main() {
  test_position_and_speed_push_target_against_travel();
  test_correction_is_limited();
  test_inversion_flips_direction();

  std::cout << "test_drift_controller PASS\n";
  return EXIT_SUCCESS;
}
