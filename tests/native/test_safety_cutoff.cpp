#include <cassert>
#include <cstdlib>
#include <iostream>

#include "../../SelfBalanceRobot/SafetyCutoff.h"

static void test_sustained_large_output_trips_after_window() {
  SafetyCutoff cutoff;
  cutoff.configure(8.0f, 180, 250UL);

  assert(!cutoff.update(9.0f, 200, 1000UL));
  assert(!cutoff.update(9.0f, 200, 1249UL));
  assert(cutoff.update(9.0f, 200, 1250UL));
}

static void test_safe_sample_resets_timer() {
  SafetyCutoff cutoff;
  cutoff.configure(8.0f, 180, 250UL);

  assert(!cutoff.update(9.0f, 200, 1000UL));
  assert(!cutoff.update(2.0f, 200, 1200UL));
  assert(!cutoff.update(9.0f, 200, 1300UL));
  assert(!cutoff.update(9.0f, 200, 1549UL));
  assert(cutoff.update(9.0f, 200, 1550UL));
}

static void test_negative_direction_counts_as_unsafe() {
  SafetyCutoff cutoff;
  cutoff.configure(8.0f, 180, 250UL);

  assert(!cutoff.update(-9.0f, -200, 1000UL));
  assert(cutoff.update(-9.0f, -200, 1250UL));
}

static void test_disabled_cutoff_never_trips() {
  SafetyCutoff cutoff;
  cutoff.configure(8.0f, 180, 0UL);

  assert(!cutoff.update(20.0f, 255, 1000UL));
  assert(!cutoff.update(20.0f, 255, 5000UL));
}

int main() {
  test_sustained_large_output_trips_after_window();
  test_safe_sample_resets_timer();
  test_negative_direction_counts_as_unsafe();
  test_disabled_cutoff_never_trips();

  std::cout << "test_safety_cutoff PASS\n";
  return EXIT_SUCCESS;
}
