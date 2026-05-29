#include <cassert>
#include <cstdlib>
#include <iostream>

#include "../../SelfBalanceRobot/RobotState.h"

static SensorFrame sensor(float angleDegrees, unsigned long nowMillis) {
  SensorFrame frame;
  frame.angleDegrees = angleDegrees;
  frame.gyroFresh = true;
  frame.nowMillis = nowMillis;
  return frame;
}

static ControlCommand command(unsigned long receivedMillis) {
  ControlCommand cmd;
  cmd.receivedMillis = receivedMillis;
  return cmd;
}

static void test_arm_calibrates_and_balances() {
  RobotState state;
  state.configure(30.0f, 4.0f, 100, 150);

  ControlCommand cmd = command(0);
  cmd.arm = true;

  state.update(sensor(1.0f, 0), cmd);
  assert(state.mode() == RobotMode::Calibrating);
  assert(!state.motorsEnabled());

  state.update(sensor(3.0f, 50), cmd);
  assert(state.mode() == RobotMode::Calibrating);

  state.update(sensor(5.0f, 100), cmd);
  assert(state.mode() == RobotMode::Balancing);
  assert(state.motorsEnabled());
  assert(state.uprightAngleDegrees() == 3.0f);
}

static void test_stale_arm_command_is_rejected() {
  RobotState state;
  state.configure(30.0f, 4.0f, 100, 50);

  ControlCommand cmd = command(0);
  cmd.arm = true;

  state.update(sensor(0.0f, 51), cmd);

  assert(state.mode() == RobotMode::Disarmed);
  assert(!state.motorsEnabled());
}

static void test_stale_gyro_at_calibration_completion_faults() {
  RobotState state;
  state.configure(30.0f, 4.0f, 100, 150);

  ControlCommand cmd = command(0);
  cmd.arm = true;

  state.update(sensor(1.0f, 0), cmd);
  assert(state.mode() == RobotMode::Calibrating);

  SensorFrame stale = sensor(3.0f, 100);
  stale.gyroFresh = false;
  state.update(stale, cmd);

  assert(state.mode() == RobotMode::Fault);
  assert(!state.motorsEnabled());
}

static void test_wide_calibration_sample_count_does_not_wrap() {
  RobotState state;
  state.configure(30.0f, 4.0f, 70000, 100000);

  ControlCommand cmd = command(0);
  cmd.arm = true;

  for (unsigned long now = 0; now <= 70000; ++now) {
    state.update(sensor(1.0f, now), cmd);
  }

  assert(state.mode() == RobotMode::Balancing);
  assert(state.uprightAngleDegrees() == 1.0f);
}

static void test_stop_disarms_immediately() {
  RobotState state;
  state.configure(30.0f, 4.0f, 0, 50);

  ControlCommand cmd = command(0);
  cmd.arm = true;
  state.update(sensor(0.0f, 0), cmd);
  assert(state.mode() == RobotMode::Balancing);

  cmd.stop = true;
  state.update(sensor(45.0f, 1), cmd);

  assert(state.mode() == RobotMode::Disarmed);
  assert(!state.motorsEnabled());
}

static void test_fall_faults_when_balancing() {
  RobotState state;
  state.configure(20.0f, 4.0f, 0, 50);

  ControlCommand cmd = command(0);
  cmd.arm = true;
  state.update(sensor(0.0f, 0), cmd);
  assert(state.mode() == RobotMode::Balancing);

  state.update(sensor(25.0f, 1), cmd);

  assert(state.mode() == RobotMode::Fault);
  assert(!state.motorsEnabled());
}

static void test_calibration_faults_when_robot_moves_too_much() {
  RobotState state;
  state.configure(30.0f, 4.0f, 100, 150);

  ControlCommand cmd = command(0);
  cmd.arm = true;

  state.update(sensor(1.0f, 0), cmd);
  assert(state.mode() == RobotMode::Calibrating);

  state.update(sensor(6.1f, 50), cmd);

  assert(state.mode() == RobotMode::Fault);
  assert(!state.motorsEnabled());
}

static void test_calibration_faults_when_arm_expires_before_finish() {
  RobotState state;
  state.configure(30.0f, 4.0f, 100, 50);

  ControlCommand cmd = command(0);
  cmd.arm = true;

  state.update(sensor(0.0f, 0), cmd);
  assert(state.mode() == RobotMode::Calibrating);

  state.update(sensor(0.5f, 100), cmd);

  assert(state.mode() == RobotMode::Fault);
  assert(!state.motorsEnabled());
}

static void test_calibration_faults_when_startup_angle_is_too_tilted() {
  RobotState state;
  state.configure(30.0f, 4.0f, 100, 150);

  ControlCommand cmd = command(0);
  cmd.arm = true;

  state.update(sensor(15.0f, 0), cmd);
  assert(state.mode() == RobotMode::Calibrating);

  state.update(sensor(15.5f, 100), cmd);

  assert(state.mode() == RobotMode::Fault);
  assert(!state.motorsEnabled());
}

static void test_start_balancing_at_sets_absolute_upright_angle() {
  RobotState state;
  state.configure(30.0f, 4.0f, 100, 150);

  assert(state.startBalancingAt(0.7f));

  assert(state.mode() == RobotMode::Balancing);
  assert(state.motorsEnabled());
  assert(state.uprightAngleDegrees() == 0.7f);
}

static void test_start_balancing_at_is_ignored_when_not_disarmed() {
  RobotState state;
  state.configure(30.0f, 4.0f, 100, 150);

  assert(state.startBalancingAt(0.7f));
  assert(!state.startBalancingAt(2.0f));

  assert(state.uprightAngleDegrees() == 0.7f);
}

static void test_start_balancing_at_rejects_tilted_startup_angle() {
  RobotState state;
  state.configure(30.0f, 4.0f, 100, 150);

  assert(!state.startBalancingAt(15.0f));

  assert(state.mode() == RobotMode::Disarmed);
  assert(!state.motorsEnabled());
}

int main() {
  test_arm_calibrates_and_balances();
  test_stale_arm_command_is_rejected();
  test_stale_gyro_at_calibration_completion_faults();
  test_wide_calibration_sample_count_does_not_wrap();
  test_stop_disarms_immediately();
  test_fall_faults_when_balancing();
  test_calibration_faults_when_robot_moves_too_much();
  test_calibration_faults_when_arm_expires_before_finish();
  test_calibration_faults_when_startup_angle_is_too_tilted();
  test_start_balancing_at_sets_absolute_upright_angle();
  test_start_balancing_at_is_ignored_when_not_disarmed();
  test_start_balancing_at_rejects_tilted_startup_angle();

  std::cout << "test_robot_state PASS\n";
  return EXIT_SUCCESS;
}
