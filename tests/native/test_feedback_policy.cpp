#include <cassert>
#include <cstdlib>
#include <iostream>

#include "../../SelfBalanceRobot/FeedbackPolicy.h"

static void test_speed_features_force_full_refresh_every_tick() {
  FeedbackPolicy policy;
  policy.configure(5);

  FeedbackRequest request;
  request.speedTargetCorrectionEnabled = true;
  request.speedDampingEnabled = false;
  request.forceFullRefresh = false;

  assert(policy.nextMode(request) == MotorFeedbackMode::Full);
  assert(policy.nextMode(request) == MotorFeedbackMode::Full);
}

static void test_disabled_speed_features_use_periodic_full_refresh() {
  FeedbackPolicy policy;
  policy.configure(3);

  FeedbackRequest request;
  request.speedTargetCorrectionEnabled = false;
  request.speedDampingEnabled = false;
  request.forceFullRefresh = false;

  assert(policy.nextMode(request) == MotorFeedbackMode::Full);
  assert(policy.nextMode(request) == MotorFeedbackMode::PositionAndPwm);
  assert(policy.nextMode(request) == MotorFeedbackMode::PositionAndPwm);
  assert(policy.nextMode(request) == MotorFeedbackMode::Full);
}

static void test_forced_refresh_does_not_break_periodic_count() {
  FeedbackPolicy policy;
  policy.configure(4);

  FeedbackRequest request;
  request.speedTargetCorrectionEnabled = false;
  request.speedDampingEnabled = false;
  request.forceFullRefresh = false;

  assert(policy.nextMode(request) == MotorFeedbackMode::Full);
  assert(policy.nextMode(request) == MotorFeedbackMode::PositionAndPwm);
  request.forceFullRefresh = true;
  assert(policy.nextMode(request) == MotorFeedbackMode::Full);
  request.forceFullRefresh = false;
  assert(policy.nextMode(request) == MotorFeedbackMode::PositionAndPwm);
  assert(policy.nextMode(request) == MotorFeedbackMode::Full);
}

int main() {
  test_speed_features_force_full_refresh_every_tick();
  test_disabled_speed_features_use_periodic_full_refresh();
  test_forced_refresh_does_not_break_periodic_count();

  std::cout << "test_feedback_policy PASS\n";
  return EXIT_SUCCESS;
}
