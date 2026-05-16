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
  state.configure(30.0f, 20.0f, 100, 50);

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
  state.configure(30.0f, 20.0f, 100, 50);

  ControlCommand cmd = command(0);
  cmd.arm = true;

  state.update(sensor(0.0f, 51), cmd);

  assert(state.mode() == RobotMode::Disarmed);
  assert(!state.motorsEnabled());
}

static void test_stale_gyro_at_calibration_completion_faults() {
  RobotState state;
  state.configure(30.0f, 20.0f, 100, 50);

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
  state.configure(30.0f, 20.0f, 70000, 100000);

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
  state.configure(30.0f, 20.0f, 0, 50);

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
  state.configure(20.0f, 20.0f, 0, 50);

  ControlCommand cmd = command(0);
  cmd.arm = true;
  state.update(sensor(0.0f, 0), cmd);
  assert(state.mode() == RobotMode::Balancing);

  state.update(sensor(25.0f, 1), cmd);

  assert(state.mode() == RobotMode::Fault);
  assert(!state.motorsEnabled());
}

static void test_obstacle_blocks_forward_only() {
  RobotState state;
  state.configure(30.0f, 20.0f, 0, 50);

  ControlCommand cmd = command(0);
  cmd.arm = true;
  state.update(sensor(0.0f, 0), cmd);

  cmd.driveEnabled = true;
  cmd.forward = 40;
  cmd.turn = 12;
  cmd.receivedMillis = 1;
  SensorFrame frame = sensor(0.0f, 1);
  state.update(frame, cmd);
  assert(state.mode() == RobotMode::Drive);

  frame.distanceCm = 10.0f;
  frame.ultrasonicFresh = true;
  ControlCommand safe = state.safeCommand(cmd, frame);

  assert(safe.forward == 0);
  assert(safe.turn == 12);

  cmd.forward = -40;
  safe = state.safeCommand(cmd, frame);
  assert(safe.forward == -40);
  assert(safe.turn == 12);
}

static void test_stale_safe_command_zeros_drive_inputs() {
  RobotState state;
  state.configure(30.0f, 20.0f, 0, 50);

  ControlCommand cmd = command(0);
  cmd.arm = true;
  state.update(sensor(0.0f, 0), cmd);

  cmd.driveEnabled = true;
  cmd.forward = 40;
  cmd.turn = 12;
  cmd.receivedMillis = 1;
  SensorFrame frame = sensor(0.0f, 1);
  state.update(frame, cmd);
  assert(state.mode() == RobotMode::Drive);

  frame.nowMillis = 52;
  ControlCommand safe = state.safeCommand(cmd, frame);

  assert(safe.forward == 0);
  assert(safe.turn == 0);
}

int main() {
  test_arm_calibrates_and_balances();
  test_stale_arm_command_is_rejected();
  test_stale_gyro_at_calibration_completion_faults();
  test_wide_calibration_sample_count_does_not_wrap();
  test_stop_disarms_immediately();
  test_fall_faults_when_balancing();
  test_obstacle_blocks_forward_only();
  test_stale_safe_command_zeros_drive_inputs();

  std::cout << "test_robot_state PASS\n";
  return EXIT_SUCCESS;
}
