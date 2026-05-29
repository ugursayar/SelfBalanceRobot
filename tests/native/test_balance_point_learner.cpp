#include <cassert>
#include <climits>
#include <cstdint>
#include <cstdlib>
#include <iostream>

#include "../../SelfBalanceRobot/BalancePointLearner.h"

static SensorFrame frame(float angle, float rate, unsigned long now,
                         bool fresh = true) {
  SensorFrame f;
  f.angleDegrees = angle;
  f.angleRateDegPerSec = rate;
  f.nowMillis = now;
  f.gyroFresh = fresh;
  return f;
}

static BalancePointLearner configured() {
  BalancePointLearner learner;
  learner.configure(1000, 500, 2000, 1.0f, 5.0f, 40, 0.25f);
  learner.reset(0.0f, 0);
  return learner;
}

static void test_rejects_before_settle_time() {
  BalancePointLearner learner = configured();

  BalanceLearningResult result =
      learner.update(frame(1.0f, 0.0f, 900), 1.0f, 10, 900);

  assert(!result.shouldSave);
}

static void test_rejects_unstable_angle_rate_and_motor_output() {
  BalancePointLearner learner = configured();

  assert(!learner.update(frame(3.0f, 0.0f, 1200), 1.0f, 10, 1200).shouldSave);
  assert(!learner.update(frame(1.0f, 8.0f, 1800), 1.0f, 10, 1800).shouldSave);
  assert(!learner.update(frame(1.0f, 0.0f, 2400), 1.0f, 80, 2400).shouldSave);
}

static void test_rejects_int16_min_motor_output() {
  BalancePointLearner learner = configured();

  assert(!learner.update(frame(1.0f, 0.0f, 1200), 1.0f, INT16_MIN, 1200)
              .shouldSave);
  assert(!learner.update(frame(1.0f, 0.0f, 1700), 1.0f, INT16_MIN, 1700)
              .shouldSave);
}

static void test_negative_thresholds_are_treated_as_magnitudes() {
  BalancePointLearner learner;
  learner.configure(1000, 500, 2000, -1.0f, -5.0f, -40, 0.25f);
  learner.reset(0.0f, 0);

  assert(!learner.update(frame(1.0f, -5.0f, 1200), 1.0f, -40, 1200)
              .shouldSave);
  BalanceLearningResult result =
      learner.update(frame(1.0f, -5.0f, 1700), 1.0f, -40, 1700);

  assert(result.shouldSave);
  assert(result.balancePointDegrees == 0.25f);
}

static void test_rejects_stale_gyro() {
  BalancePointLearner learner = configured();

  assert(!learner.update(frame(1.0f, 0.0f, 1200, false), 1.0f, 10, 1200)
              .shouldSave);
  assert(!learner.update(frame(1.0f, 0.0f, 1700), 1.0f, 10, 1700).shouldSave);
}

static void test_saves_smoothed_point_after_stable_window() {
  BalancePointLearner learner = configured();

  assert(!learner.update(frame(1.0f, 0.0f, 1200), 1.0f, 10, 1200).shouldSave);
  BalanceLearningResult result =
      learner.update(frame(1.0f, 0.0f, 1700), 1.0f, 10, 1700);

  assert(result.shouldSave);
  assert(result.balancePointDegrees == 0.25f);
}

static void test_negative_alpha_clamps_to_stored_point() {
  BalancePointLearner learner;
  learner.configure(0, 0, 0, 1.0f, 5.0f, 40, -0.5f);
  learner.reset(2.0f, 0);

  assert(!learner.update(frame(6.0f, 0.0f, 0), 6.0f, 10, 0).shouldSave);
  BalanceLearningResult result =
      learner.update(frame(6.0f, 0.0f, 0), 6.0f, 10, 0);

  assert(result.shouldSave);
  assert(result.balancePointDegrees == 2.0f);
}

static void test_alpha_above_one_clamps_to_active_point() {
  BalancePointLearner learner;
  learner.configure(0, 0, 0, 1.0f, 5.0f, 40, 1.5f);
  learner.reset(2.0f, 0);

  assert(!learner.update(frame(6.0f, 0.0f, 0), 6.0f, 10, 0).shouldSave);
  BalanceLearningResult result =
      learner.update(frame(6.0f, 0.0f, 0), 6.0f, 10, 0);

  assert(result.shouldSave);
  assert(result.balancePointDegrees == 6.0f);
}

static void test_min_write_interval_prevents_repeated_writes() {
  BalancePointLearner learner = configured();

  assert(!learner.update(frame(1.0f, 0.0f, 1200), 1.0f, 10, 1200).shouldSave);
  assert(learner.update(frame(1.0f, 0.0f, 1700), 1.0f, 10, 1700).shouldSave);
  assert(!learner.update(frame(1.0f, 0.0f, 1900), 1.0f, 10, 1900).shouldSave);
  assert(learner.update(frame(1.0f, 0.0f, 3700), 1.0f, 10, 3700).shouldSave);
}

static void test_reset_does_not_bypass_min_write_interval() {
  BalancePointLearner learner = configured();

  assert(!learner.update(frame(1.0f, 0.0f, 1200), 1.0f, 10, 1200).shouldSave);
  assert(learner.update(frame(1.0f, 0.0f, 1700), 1.0f, 10, 1700).shouldSave);

  learner.reset(0.25f, 1800);

  assert(!learner.update(frame(2.0f, 0.0f, 3000), 2.0f, 10, 3000).shouldSave);
  assert(!learner.update(frame(2.0f, 0.0f, 3500), 2.0f, 10, 3500).shouldSave);
  assert(learner.update(frame(2.0f, 0.0f, 3700), 2.0f, 10, 3700).shouldSave);
}

static void test_stable_window_survives_millis_wraparound() {
  BalancePointLearner learner;
  learner.configure(0, 20, 0, 1.0f, 5.0f, 40, 1.0f);
  learner.reset(0.0f, ULONG_MAX - 10);

  assert(!learner.update(frame(1.0f, 0.0f, ULONG_MAX - 5), 1.0f, 10,
                         ULONG_MAX - 5)
              .shouldSave);
  BalanceLearningResult result =
      learner.update(frame(1.0f, 0.0f, 14), 1.0f, 10, 14);

  assert(result.shouldSave);
  assert(result.balancePointDegrees == 1.0f);
}

static void test_min_write_interval_survives_millis_wraparound() {
  BalancePointLearner learner;
  learner.configure(0, 0, 5, 1.0f, 5.0f, 40, 1.0f);
  learner.reset(0.0f, ULONG_MAX - 10);

  assert(!learner.update(frame(1.0f, 0.0f, ULONG_MAX - 2), 1.0f, 10,
                         ULONG_MAX - 2)
              .shouldSave);
  assert(learner.update(frame(1.0f, 0.0f, ULONG_MAX - 2), 1.0f, 10,
                        ULONG_MAX - 2)
             .shouldSave);
  assert(!learner.update(frame(2.0f, 0.0f, 1), 2.0f, 10, 1).shouldSave);
  BalanceLearningResult result =
      learner.update(frame(2.0f, 0.0f, 3), 2.0f, 10, 3);

  assert(result.shouldSave);
  assert(result.balancePointDegrees == 2.0f);
}

static void test_reset_restarts_settle_window_and_uses_new_stored_value() {
  BalancePointLearner learner = configured();
  learner.reset(2.0f, 4000);

  assert(!learner.update(frame(4.0f, 0.0f, 4900), 4.0f, 10, 4900).shouldSave);
  assert(!learner.update(frame(4.0f, 0.0f, 5200), 4.0f, 10, 5200).shouldSave);
  BalanceLearningResult result =
      learner.update(frame(4.0f, 0.0f, 5700), 4.0f, 10, 5700);

  assert(result.shouldSave);
  assert(result.balancePointDegrees == 2.5f);
}

int main() {
  test_rejects_before_settle_time();
  test_rejects_unstable_angle_rate_and_motor_output();
  test_rejects_int16_min_motor_output();
  test_negative_thresholds_are_treated_as_magnitudes();
  test_rejects_stale_gyro();
  test_saves_smoothed_point_after_stable_window();
  test_negative_alpha_clamps_to_stored_point();
  test_alpha_above_one_clamps_to_active_point();
  test_min_write_interval_prevents_repeated_writes();
  test_reset_does_not_bypass_min_write_interval();
  test_stable_window_survives_millis_wraparound();
  test_min_write_interval_survives_millis_wraparound();
  test_reset_restarts_settle_window_and_uses_new_stored_value();

  std::cout << "test_balance_point_learner PASS\n";
  return EXIT_SUCCESS;
}
