#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <iostream>

#define private public
#include "../../SelfBalanceRobot/BalanceController.h"
#undef private

static void test_output_sign_and_limit() {
  BalanceController controller;
  controller.setTunings(10.0f, 0.0f, 0.0f);
  controller.setOutputLimit(100);

  controller.setTargetAngle(0.0f);
  assert(controller.update(-20.0f, 0.02f) == -100);
  assert(controller.update(20.0f, 0.02f) == 100);
}

static void test_negative_limit_is_stored_positive() {
  BalanceController controller;
  controller.setTunings(10.0f, 0.0f, 0.0f);
  controller.setOutputLimit(-30);

  assert(controller.update(-20.0f, 0.02f) == -30);
}

static void test_reset_clears_integral_and_previous_error() {
  BalanceController controller;
  controller.setTunings(0.0f, 1.0f, 0.0f);
  controller.setOutputLimit(200);
  controller.setTargetAngle(10.0f);

  const int16_t first = controller.update(0.0f, 1.0f);
  const int16_t second = controller.update(5.0f, 1.0f);
  assert(first == -10);
  assert(second == -15);
  assert(controller.integral_ != 0.0f);
  assert(controller.hasPreviousMeasuredAngle_);

  controller.reset();

  assert(controller.integral_ == 0.0f);
  assert(controller.previousMeasuredAngleDegrees_ == 0.0f);
  assert(!controller.hasPreviousMeasuredAngle_);
  assert(controller.update(5.0f, 1.0f) == -5);
}

static void test_non_positive_dt_returns_zero_without_state_update() {
  BalanceController controller;
  controller.setTunings(2.0f, 1.0f, 1.0f);
  controller.setOutputLimit(200);
  controller.setTargetAngle(10.0f);

  assert(controller.update(0.0f, 0.0f) == 0);
  assert(controller.integral_ == 0.0f);
  assert(controller.previousMeasuredAngleDegrees_ == 0.0f);
  assert(!controller.hasPreviousMeasuredAngle_);

  assert(controller.update(0.0f, -0.1f) == 0);
  assert(controller.integral_ == 0.0f);
  assert(controller.previousMeasuredAngleDegrees_ == 0.0f);
  assert(!controller.hasPreviousMeasuredAngle_);

  assert(controller.update(0.0f, 1.0f) == -30);
}

static void test_integral_limit_clamps_accumulated_error() {
  BalanceController controller;
  controller.setTunings(0.0f, 10.0f, 0.0f);
  controller.setIntegralLimit(2.0f);
  controller.setOutputLimit(200);
  controller.setTargetAngle(10.0f);

  assert(controller.update(0.0f, 1.0f) == -20);
  assert(controller.update(0.0f, 1.0f) == -20);
  assert(controller.integral_ == 2.0f);
}

static void test_measured_rate_damps_motion() {
  BalanceController controller;
  controller.setTunings(10.0f, 0.0f, 1.0f);
  controller.setOutputLimit(200);
  controller.setTargetAngle(0.0f);

  assert(controller.update(0.0f, 1.0f) == 0);
  assert(controller.update(-1.0f, 1.0f) == -11);
  assert(controller.update(0.0f, 1.0f) == 1);

  controller.reset();
  assert(controller.update(1.0f, 1.0f) == 10);
  assert(controller.update(0.0f, 1.0f) == -1);
}

static void test_rate_filter_smooths_derivative_input() {
  BalanceController controller;
  controller.setTunings(0.0f, 0.0f, 10.0f);
  controller.setRateFilter(0.5f);
  controller.setOutputLimit(200);
  controller.setTargetAngle(0.0f);

  assert(controller.update(0.0f, 1.0f) == 0);
  assert(controller.update(10.0f, 1.0f) == 50);
  assert(controller.update(10.0f, 1.0f) == 25);
}

int main() {
  test_output_sign_and_limit();
  test_negative_limit_is_stored_positive();
  test_reset_clears_integral_and_previous_error();
  test_non_positive_dt_returns_zero_without_state_update();
  test_integral_limit_clamps_accumulated_error();
  test_measured_rate_damps_motion();
  test_rate_filter_smooths_derivative_input();

  std::cout << "test_balance_controller PASS\n";
  return EXIT_SUCCESS;
}
