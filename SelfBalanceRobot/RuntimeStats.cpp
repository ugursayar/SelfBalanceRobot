#include "RuntimeStats.h"

void RuntimeStats::recordBalanceTick(unsigned long loopIntervalMicros,
                                     unsigned long workMicros,
                                     unsigned long targetLoopMicros) {
  ++snapshot_.balanceTicks;
  snapshot_.lastLoopIntervalMicros = loopIntervalMicros;
  snapshot_.lastWorkMicros = workMicros;
  if (workMicros > snapshot_.maxWorkMicros) {
    snapshot_.maxWorkMicros = workMicros;
  }
  if (loopIntervalMicros > targetLoopMicros) {
    ++snapshot_.missedDeadlines;
  }
}

void RuntimeStats::recordFeedbackRefresh(bool fullRefresh) {
  if (fullRefresh) {
    ++snapshot_.fullFeedbackRefreshes;
  } else {
    ++snapshot_.lightFeedbackRefreshes;
  }
}

void RuntimeStats::recordMotorWrite() { ++snapshot_.motorWrites; }

void RuntimeStats::recordMotorStop() { ++snapshot_.motorStops; }

void RuntimeStats::recordTelemetryPrint(unsigned long telemetryMicros) {
  snapshot_.lastTelemetryMicros = telemetryMicros;
  if (telemetryMicros > snapshot_.maxTelemetryMicros) {
    snapshot_.maxTelemetryMicros = telemetryMicros;
  }
}

void RuntimeStats::resetPeaks() {
  snapshot_.maxWorkMicros = snapshot_.lastWorkMicros;
  snapshot_.maxTelemetryMicros = snapshot_.lastTelemetryMicros;
}

RuntimeStatsSnapshot RuntimeStats::snapshot() const { return snapshot_; }
