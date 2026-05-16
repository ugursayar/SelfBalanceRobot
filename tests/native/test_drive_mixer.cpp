#include <cassert>
#include <cstdlib>
#include <iostream>

#include "../../SelfBalanceRobot/config.h"
#include "../../SelfBalanceRobot/DriveMixer.h"

static void test_balance_goes_to_both_motors() {
  DriveMixer mixer;
  mixer.setLimits(160, 50, 50, 8);

  const int16_t balanceOutput =
      Config::InvertBalanceOutput ? static_cast<int16_t>(-40) : 40;
  const MotorCommand command = mixer.mix(balanceOutput, 0, 0);

  assert(command.left == -40);
  assert(command.right == -40);
}

static void test_turn_adds_opposite_offsets() {
  DriveMixer mixer;
  mixer.setLimits(160, 50, 50, 8);

  const MotorCommand command = mixer.mix(40, 0, 15);

  assert(command.left == 25);
  assert(command.right == 55);
}

static void test_deadband_and_saturation() {
  DriveMixer mixer;
  mixer.setLimits(100, 50, 50, 8);

  const MotorCommand small = mixer.mix(4, 0, 0);
  assert(small.left == 0);
  assert(small.right == 0);

  const MotorCommand saturated = mixer.mix(120, 50, 50);
  assert(saturated.left == 100);
  assert(saturated.right == 100);
}

int main() {
  test_balance_goes_to_both_motors();
  test_turn_adds_opposite_offsets();
  test_deadband_and_saturation();

  std::cout << "test_drive_mixer PASS\n";
  return EXIT_SUCCESS;
}
