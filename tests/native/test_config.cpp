#include <cassert>
#include <cstdlib>
#include <iostream>

#include "../../SelfBalanceRobot/config.h"

static void test_auto_arm_rate_gate_matches_cable_free_tuning() {
  assert(Config::AutoArmMaxRateDegPerSec == 12.0f);
}

int main() {
  test_auto_arm_rate_gate_matches_cable_free_tuning();

  std::cout << "test_config PASS\n";
  return EXIT_SUCCESS;
}
