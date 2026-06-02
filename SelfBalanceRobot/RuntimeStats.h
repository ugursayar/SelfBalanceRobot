#ifndef RUNTIME_STATS_H
#define RUNTIME_STATS_H

#include <stdint.h>

struct RuntimeStatsSnapshot {
  uint32_t balanceTicks = 0;
  unsigned long lastLoopIntervalMicros = 0;
  unsigned long lastWorkMicros = 0;
  unsigned long maxWorkMicros = 0;
  uint32_t missedDeadlines = 0;
  uint32_t fullFeedbackRefreshes = 0;
  uint32_t lightFeedbackRefreshes = 0;
  uint32_t motorWrites = 0;
  uint32_t motorStops = 0;
  unsigned long lastTelemetryMicros = 0;
  unsigned long maxTelemetryMicros = 0;
};

class RuntimeStats {
public:
  void recordBalanceTick(unsigned long loopIntervalMicros,
                         unsigned long workMicros,
                         unsigned long targetLoopMicros);
  void recordFeedbackRefresh(bool fullRefresh);
  void recordMotorWrite();
  void recordMotorStop();
  void recordTelemetryPrint(unsigned long telemetryMicros);
  void resetPeaks();
  RuntimeStatsSnapshot snapshot() const;

private:
  RuntimeStatsSnapshot snapshot_;
};

#endif
