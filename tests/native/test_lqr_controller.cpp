#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <iostream>

#include "../../SelfBalanceRobot/LqrController.h"

static bool nearlyEqual(float a, float b) {
  const float diff = a > b ? a - b : b - a;
  return diff < 1e-4f;
}

// u = K * (x - x*).  With angle gain only, leaning past the target produces a
// corrective command in the same sign convention as the PID kp term
// (positive measured-minus-target -> positive command).
static void test_angle_gain_sign_matches_pid_convention() {
  LqrController lqr;
  lqr.setGains(10.0f, 0.0f, 0.0f, 0.0f);
  lqr.setOutputLimit(200);
  lqr.setTargetAngle(0.0f);

  assert(lqr.update(2.0f, 0.0f, 0.0f, 0.0f, 0.02f) == 20);
  assert(lqr.update(-2.0f, 0.0f, 0.0f, 0.0f, 0.02f) == -20);
}

static void test_output_is_clamped_to_limit() {
  LqrController lqr;
  lqr.setGains(50.0f, 0.0f, 0.0f, 0.0f);
  lqr.setOutputLimit(80);
  lqr.setTargetAngle(0.0f);

  assert(lqr.update(10.0f, 0.0f, 0.0f, 0.0f, 0.02f) == 80);
  assert(lqr.update(-10.0f, 0.0f, 0.0f, 0.0f, 0.02f) == -80);
}

static void test_negative_limit_is_stored_positive() {
  LqrController lqr;
  lqr.setGains(50.0f, 0.0f, 0.0f, 0.0f);
  lqr.setOutputLimit(-30);
  lqr.setTargetAngle(0.0f);

  assert(lqr.update(10.0f, 0.0f, 0.0f, 0.0f, 0.02f) == 30);
}

// All four state gains contribute linearly to the command.
static void test_full_state_feedback_sums_all_terms() {
  LqrController lqr;
  lqr.setGains(2.0f, 3.0f, 0.5f, 0.1f);
  lqr.setRateFilter(0.0f);  // pass rate through unfiltered
  lqr.setOutputLimit(1000);
  lqr.setTargetAngle(1.0f);

  // angleDeviation = 4 - 1 = 3 -> 2*3 = 6
  // rate term       = 3 * 10 = 30
  // wheelPos term    = 0.5 * 8 = 4
  // wheelSpeed term  = 0.1 * 20 = 2
  // total = 42
  const int16_t output = lqr.update(4.0f, 10.0f, 8.0f, 20.0f, 0.02f);
  assert(output == 42);
}

// The gain schedule scales the angle term down near the target and ramps back
// to full at the window edge, leaving big-lean recovery untouched.
static void test_gain_schedule_softens_small_corrections() {
  LqrController lqr;
  lqr.setGains(10.0f, 0.0f, 0.0f, 0.0f);
  lqr.setGainSchedule(4.0f, 0.5f);
  lqr.setOutputLimit(1000);
  lqr.setTargetAngle(0.0f);

  // deviation 2 (half the window): scale = 0.5 + 0.5*(2/4) = 0.75 -> gain 7.5,
  // output = 7.5 * 2 = 15 (vs 20 unscheduled).
  assert(lqr.update(2.0f, 0.0f, 0.0f, 0.0f, 0.02f) == 15);
  // Sign is preserved on the other side.
  assert(lqr.update(-2.0f, 0.0f, 0.0f, 0.0f, 0.02f) == -15);
  // deviation 4 (window edge): full gain -> 40.
  assert(lqr.update(4.0f, 0.0f, 0.0f, 0.0f, 0.02f) == 40);
  // deviation 8 (beyond the window): still full gain -> 80.
  assert(lqr.update(8.0f, 0.0f, 0.0f, 0.0f, 0.02f) == 80);
}

static void test_default_schedule_is_linear() {
  LqrController lqr;
  lqr.setGains(10.0f, 0.0f, 0.0f, 0.0f);  // no setGainSchedule -> full gain
  lqr.setOutputLimit(1000);
  lqr.setTargetAngle(0.0f);
  assert(lqr.update(2.0f, 0.0f, 0.0f, 0.0f, 0.02f) == 20);
}

static void test_rate_filter_smooths_rate_term() {
  LqrController lqr;
  lqr.setGains(0.0f, 1.0f, 0.0f, 0.0f);
  lqr.setRateFilter(0.5f);
  lqr.setOutputLimit(1000);
  lqr.setTargetAngle(0.0f);

  // filtered = 0.5*0 + 0.5*100 = 50
  assert(lqr.update(0.0f, 100.0f, 0.0f, 0.0f, 0.02f) == 50);
  assert(nearlyEqual(lqr.lastMeasuredAngleRateDegreesPerSecond(), 50.0f));
  // filtered = 0.5*50 + 0.5*100 = 75
  assert(lqr.update(0.0f, 100.0f, 0.0f, 0.0f, 0.02f) == 75);
}

static void test_non_positive_dt_returns_zero() {
  LqrController lqr;
  lqr.setGains(10.0f, 10.0f, 10.0f, 10.0f);
  lqr.setOutputLimit(200);
  lqr.setTargetAngle(0.0f);

  assert(lqr.update(5.0f, 5.0f, 5.0f, 5.0f, 0.0f) == 0);
  assert(lqr.update(5.0f, 5.0f, 5.0f, 5.0f, -0.1f) == 0);
}

static void test_reset_clears_filter_state() {
  LqrController lqr;
  lqr.setGains(0.0f, 1.0f, 0.0f, 0.0f);
  lqr.setRateFilter(0.5f);
  lqr.setOutputLimit(1000);
  lqr.setTargetAngle(0.0f);

  lqr.update(0.0f, 100.0f, 0.0f, 0.0f, 0.02f);  // filtered -> 50
  lqr.reset();
  // After reset the filter restarts from zero: 0.5*0 + 0.5*100 = 50 again.
  assert(lqr.update(0.0f, 100.0f, 0.0f, 0.0f, 0.02f) == 50);
}

static void test_error_uses_pid_sign_convention_for_telemetry() {
  LqrController lqr;
  lqr.setGains(1.0f, 0.0f, 0.0f, 0.0f);
  lqr.setOutputLimit(200);
  lqr.setTargetAngle(2.0f);

  lqr.update(5.0f, 0.0f, 0.0f, 0.0f, 0.02f);
  // errorDegrees = target - measured = 2 - 5 = -3 (matches BalanceController).
  assert(nearlyEqual(lqr.lastErrorDegrees(), -3.0f));
}

int main() {
  test_angle_gain_sign_matches_pid_convention();
  test_output_is_clamped_to_limit();
  test_negative_limit_is_stored_positive();
  test_full_state_feedback_sums_all_terms();
  test_gain_schedule_softens_small_corrections();
  test_default_schedule_is_linear();
  test_rate_filter_smooths_rate_term();
  test_non_positive_dt_returns_zero();
  test_reset_clears_filter_state();
  test_error_uses_pid_sign_convention_for_telemetry();

  std::cout << "test_lqr_controller PASS\n";
  return EXIT_SUCCESS;
}
