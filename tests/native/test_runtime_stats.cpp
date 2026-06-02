#include <cassert>
#include <cstdlib>
#include <iostream>

#include "../../SelfBalanceRobot/RuntimeStats.h"

static void test_records_loop_work_and_peak() {
  RuntimeStats stats;

  stats.recordBalanceTick(9000, 4000, 10000);
  RuntimeStatsSnapshot snap = stats.snapshot();

  assert(snap.balanceTicks == 1);
  assert(snap.lastLoopIntervalMicros == 9000);
  assert(snap.lastWorkMicros == 4000);
  assert(snap.maxWorkMicros == 4000);
  assert(snap.missedDeadlines == 0);

  stats.recordBalanceTick(11000, 7000, 10000);
  snap = stats.snapshot();

  assert(snap.balanceTicks == 2);
  assert(snap.lastLoopIntervalMicros == 11000);
  assert(snap.lastWorkMicros == 7000);
  assert(snap.maxWorkMicros == 7000);
  assert(snap.missedDeadlines == 1);
}

static void test_records_subsystem_counts() {
  RuntimeStats stats;

  stats.recordFeedbackRefresh(true);
  stats.recordFeedbackRefresh(false);
  stats.recordMotorWrite();
  stats.recordMotorWrite();
  stats.recordMotorStop();
  stats.recordTelemetryPrint(1200);
  stats.recordTelemetryPrint(800);

  const RuntimeStatsSnapshot snap = stats.snapshot();

  assert(snap.fullFeedbackRefreshes == 1);
  assert(snap.lightFeedbackRefreshes == 1);
  assert(snap.motorWrites == 2);
  assert(snap.motorStops == 1);
  assert(snap.lastTelemetryMicros == 800);
  assert(snap.maxTelemetryMicros == 1200);
}

static void test_peak_reset_keeps_cumulative_counts() {
  RuntimeStats stats;

  stats.recordBalanceTick(12000, 9000, 10000);
  stats.recordTelemetryPrint(600);
  stats.resetPeaks();

  const RuntimeStatsSnapshot snap = stats.snapshot();

  assert(snap.balanceTicks == 1);
  assert(snap.missedDeadlines == 1);
  assert(snap.lastWorkMicros == 9000);
  assert(snap.maxWorkMicros == 9000);
  assert(snap.lastTelemetryMicros == 600);
  assert(snap.maxTelemetryMicros == 600);
}

int main() {
  test_records_loop_work_and_peak();
  test_records_subsystem_counts();
  test_peak_reset_keeps_cumulative_counts();

  std::cout << "test_runtime_stats PASS\n";
  return EXIT_SUCCESS;
}
