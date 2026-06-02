#include <cassert>
#include <cstdlib>
#include <iostream>

#include "../../SelfBalanceRobot/BalancePipeline.h"

static SensorFrame sensor(float angle, float rate, unsigned long nowMillis) {
  SensorFrame frame;
  frame.angleDegrees = angle;
  frame.angleRateDegPerSec = rate;
  frame.nowMillis = nowMillis;
  frame.gyroFresh = true;
  return frame;
}

static WheelFeedback wheels(float speedRpm, long positionDegrees) {
  WheelFeedback feedback;
  feedback.averageSpeedRpm = speedRpm;
  feedback.averagePositionDegrees = positionDegrees;
  return feedback;
}

static BalancePipelineInput inputAt(unsigned long nowMillis) {
  BalancePipelineInput input;
  input.frame = sensor(1.0f, 0.0f, nowMillis);
  input.wheelFeedback = wheels(0.0f, 0);
  input.uprightAngleDegrees = 2.0f;
  input.activeBalancePointDegrees = 0.7f;
  input.currentTrimDegrees = -2.0f;
  input.balancingStartMillis = 1000;
  input.balanceSessionUsesPersistedPoint = false;
  input.dtSeconds = 0.01f;
  return input;
}

static BalanceController controllerWith(float kp, float ki, float kd) {
  BalanceController controller;
  controller.setTunings(kp, ki, kd);
  controller.setIntegralLimit(30.0f);
  controller.setRateFilter(0.0f);
  controller.setOutputLimit(255);
  return controller;
}

static void test_manual_target_ramps_from_upright_to_trimmed_target() {
  BalancePipeline pipeline;
  BalanceController controller = controllerWith(0.0f, 0.0f, 0.0f);

  BalancePipelineInput input = inputAt(1750);
  BalancePipelineOutput output = pipeline.update(input, controller);

  assert(output.baseTargetDegrees == 0.0f);
  assert(output.targetAngleDegrees > 0.99f);
  assert(output.targetAngleDegrees < 1.01f);
}

static void
test_persisted_session_uses_active_balance_point_without_manual_trim() {
  BalancePipeline pipeline;
  BalanceController controller = controllerWith(0.0f, 0.0f, 0.0f);

  BalancePipelineInput input = inputAt(1100);
  input.balanceSessionUsesPersistedPoint = true;
  input.activeBalancePointDegrees = 0.75f;
  BalancePipelineOutput output = pipeline.update(input, controller);

  assert(output.baseTargetDegrees == 0.75f);
  assert(output.targetAngleDegrees == 0.75f);
}

static void test_minimum_boost_preserves_pid_direction() {
  BalancePipeline pipeline;
  BalanceController controller = controllerWith(1.0f, 0.0f, 0.0f);

  BalancePipelineInput input = inputAt(3000);
  input.frame.angleDegrees = 0.0f;
  input.uprightAngleDegrees = 0.0f;
  input.currentTrimDegrees = 2.0f;

  BalancePipelineOutput output = pipeline.update(input, controller);

  assert(output.rawBalanceOutput == -2);
  assert(output.balanceOutput <= -Config::MinBalanceMotorCommand);
}

static void test_large_lean_boost_adds_extra_recovery_command() {
  BalancePipeline pipeline;
  BalanceController controller = controllerWith(0.0f, 0.0f, 0.0f);

  BalancePipelineInput input = inputAt(3000);
  input.frame.angleDegrees = 0.0f;
  input.uprightAngleDegrees = 0.0f;
  input.currentTrimDegrees = 3.0f;

  BalancePipelineOutput output = pipeline.update(input, controller);

  assert(output.rawBalanceOutput == 0);
  assert(output.balanceOutput == -6);
}

static void test_disabled_wheel_speed_terms_do_not_change_output() {
  BalancePipeline pipeline;
  BalanceController controller = controllerWith(0.0f, 0.0f, 0.0f);

  BalancePipelineInput input = inputAt(3000);
  input.wheelFeedback = wheels(120.0f, 0);

  BalancePipelineOutput output = pipeline.update(input, controller);

  assert(output.balanceOutput == 0);
}

int main() {
  test_manual_target_ramps_from_upright_to_trimmed_target();
  test_persisted_session_uses_active_balance_point_without_manual_trim();
  test_minimum_boost_preserves_pid_direction();
  test_large_lean_boost_adds_extra_recovery_command();
  test_disabled_wheel_speed_terms_do_not_change_output();

  std::cout << "test_balance_pipeline PASS\n";
  return EXIT_SUCCESS;
}
