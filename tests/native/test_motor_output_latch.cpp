#include <cassert>
#include <cstdlib>
#include <iostream>

#include "../../SelfBalanceRobot/MotorOutputLatch.h"

static MotorCommand command(int16_t value) {
  MotorCommand output;
  output.left = value;
  output.right = value;
  return output;
}

static void test_first_nonzero_command_should_write() {
  MotorOutputLatch latch;

  assert(latch.shouldWrite(command(20)));
  assert(!latch.shouldWrite(command(20)));
}

static void test_changed_command_should_write() {
  MotorOutputLatch latch;

  assert(latch.shouldWrite(command(20)));
  assert(latch.shouldWrite(command(21)));
  assert(!latch.shouldWrite(command(21)));
}

static void test_stop_is_emitted_once_until_reset_or_new_write() {
  MotorOutputLatch latch;

  assert(latch.shouldStop());
  assert(!latch.shouldStop());
  assert(latch.shouldWrite(command(10)));
  assert(latch.shouldStop());
  assert(!latch.shouldStop());
}

static void test_reset_forces_next_write_and_stop() {
  MotorOutputLatch latch;

  assert(latch.shouldWrite(command(10)));
  assert(latch.shouldStop());
  latch.reset();
  assert(latch.shouldWrite(command(10)));
  assert(latch.shouldStop());
}

int main() {
  test_first_nonzero_command_should_write();
  test_changed_command_should_write();
  test_stop_is_emitted_once_until_reset_or_new_write();
  test_reset_forces_next_write_and_stop();

  std::cout << "test_motor_output_latch PASS\n";
  return EXIT_SUCCESS;
}
