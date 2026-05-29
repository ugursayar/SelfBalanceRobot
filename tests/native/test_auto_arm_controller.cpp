#include <cassert>
#include <cstdlib>
#include <iostream>

#include "../../SelfBalanceRobot/AutoArmController.h"

static SensorFrame frame(float angle, float rate, unsigned long now,
                         bool fresh = true) {
  SensorFrame f;
  f.angleDegrees = angle;
  f.angleRateDegPerSec = rate;
  f.nowMillis = now;
  f.gyroFresh = fresh;
  return f;
}

static AutoArmController configured() {
  AutoArmController controller;
  controller.configure(2.0f, 5.0f, 500);
  controller.setTargetBalancePoint(0.7f);
  return controller;
}

static void test_does_not_trigger_outside_angle_window() {
  AutoArmController controller = configured();

  assert(!controller.update(frame(3.0f, 0.0f, 0)));
  assert(!controller.update(frame(3.0f, 0.0f, 600)));
}

static void test_does_not_trigger_when_rate_is_too_high() {
  AutoArmController controller = configured();

  assert(!controller.update(frame(0.8f, 8.0f, 0)));
  assert(!controller.update(frame(0.8f, 8.0f, 600)));
}

static void test_triggers_after_stillness_duration() {
  AutoArmController controller = configured();

  assert(!controller.update(frame(0.8f, 1.0f, 1000)));
  assert(!controller.update(frame(0.6f, 0.5f, 1300)));
  assert(controller.update(frame(0.7f, 0.0f, 1500)));
}

static void test_resets_candidate_when_motion_breaks_stillness() {
  AutoArmController controller = configured();

  assert(!controller.update(frame(0.8f, 0.0f, 0)));
  assert(!controller.update(frame(0.9f, 10.0f, 300)));
  assert(!controller.update(frame(0.8f, 0.0f, 600)));
  assert(!controller.update(frame(0.8f, 0.0f, 900)));
  assert(controller.update(frame(0.8f, 0.0f, 1100)));
}

static void test_stop_cooldown_suppresses_auto_arm_until_expired() {
  AutoArmController controller = configured();
  controller.suppressUntil(1000, 1000);

  assert(!controller.update(frame(0.7f, 0.0f, 1200)));
  assert(!controller.update(frame(0.7f, 0.0f, 1900)));
  assert(!controller.update(frame(0.7f, 0.0f, 2000)));
  assert(controller.update(frame(0.7f, 0.0f, 2500)));
}

static void test_expired_cooldown_does_not_suppress_after_timer_wrap() {
  AutoArmController controller = configured();
  controller.suppressUntil(1000, 1000);

  assert(!controller.update(frame(0.7f, 0.0f, 2000)));
  assert(controller.update(frame(0.7f, 0.0f, 2500)));
  controller.reset();

  assert(!controller.update(frame(0.7f, 0.0f, 0)));
  assert(controller.update(frame(0.7f, 0.0f, 500)));
}

int main() {
  test_does_not_trigger_outside_angle_window();
  test_does_not_trigger_when_rate_is_too_high();
  test_triggers_after_stillness_duration();
  test_resets_candidate_when_motion_breaks_stillness();
  test_stop_cooldown_suppresses_auto_arm_until_expired();
  test_expired_cooldown_does_not_suppress_after_timer_wrap();

  std::cout << "test_auto_arm_controller PASS\n";
  return EXIT_SUCCESS;
}
