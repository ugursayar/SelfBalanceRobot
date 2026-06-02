# Performance Simplification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add measured runtime instrumentation and behavior-preserving simplifications so the robot's 100 Hz balance loop is easier to reason about and cheaper to run.

**Architecture:** Keep `SelfBalanceRobot.ino` as the hardware coordinator, but move pure control-path logic, motor-output gating, feedback scheduling, and telemetry formatting into focused testable modules. Start with timing counters so each later simplification can be checked against loop work time and missed-deadline counts without changing PID math, motor signs, balance targets, or auto-arm semantics.

**Tech Stack:** Arduino C++11 for MegaPi/Mega 2560, Makeblock `MeMegaPi`, AVR `micros()`/`millis()`, native C++ tests through `tests/native/Makefile`, Arduino CLI compile for `arduino:avr:mega:cpu=atmega2560`.

---

## Baseline

Before implementing this plan, confirm the active working tree is the same code shape this plan was written against:

- `SelfBalanceRobot/SelfBalanceRobot.ino` owns the main 100 Hz scheduler and is about 823 lines.
- `Config::BalanceLoopMicros` is `10000UL`.
- `Config::EnableDebugSerial` and `Config::EnableBluetoothTestControl` are both enabled.
- `ROBOT_BLUETOOTH_SERIAL` maps to `Serial3`.
- Native tests pass with `make -C tests/native all`.
- Arduino compile passes with `arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 SelfBalanceRobot`.

Do not reset or discard unrelated dirty working-tree changes. Commit only the files changed by each task.

## File Structure

- Create `SelfBalanceRobot/RuntimeStats.h` and `.cpp`
  - Records loop work duration, max work duration, missed loop deadlines, balance tick count, motor write count, motor stop count, feedback refresh count, and telemetry print duration.
- Create `tests/native/test_runtime_stats.cpp`
  - Tests wraparound-safe tick duration and missed-deadline accounting.
- Create `SelfBalanceRobot/BalancePipeline.h` and `.cpp`
  - Moves target calculation, PID update, output shaping, travel hold correction, and wheel damping out of `SelfBalanceRobot.ino`.
- Create `tests/native/test_balance_pipeline.cpp`
  - Tests target ramping, persisted-point target semantics, minimum command boost, large-lean boost, and wheel damping disabled behavior.
- Create `SelfBalanceRobot/MotorOutputLatch.h` and `.cpp`
  - Prevents repeated identical PWM writes and repeated idle stops while still allowing explicit reset after fault/stop.
- Create `tests/native/test_motor_output_latch.cpp`
  - Tests first write, duplicate suppression, changed write, stop gating, and reset behavior.
- Create `SelfBalanceRobot/FeedbackPolicy.h` and `.cpp`
  - Decides when full encoder speed refresh is needed versus lighter position/PWM refresh.
- Create `tests/native/test_feedback_policy.cpp`
  - Tests full refresh every tick when speed terms are enabled, periodic refresh when disabled, and forced refresh for telemetry/status.
- Create `SelfBalanceRobot/TelemetryFormatter.h`
  - Header-only template formatter for status, debug, and Bluetooth telemetry so native tests can use a capture object while the sketch passes Arduino `Stream`.
- Create `tests/native/test_telemetry_formatter.cpp`
  - Tests stable field names and avoids duplicate status/debug format drift.
- Modify `SelfBalanceRobot/Motors.h` and `.cpp`
  - Adds a feedback mode so speed computation can be skipped on ticks that only need position and PWM feedback.
- Modify `SelfBalanceRobot/SelfBalanceRobot.ino`
  - Wires the new modules into setup, loop, command handling, status output, debug output, and motor output.
- Modify `SelfBalanceRobot/config.h`
  - Adds explicit instrumentation and feedback-refresh constants.
- Modify `tests/native/Makefile`
  - Builds the new native tests and adjusts the Arduino stub only where needed.
- Modify `docs/bring-up.md`
  - Documents runtime counters, expected interpretation, and the `Serial3` Bluetooth control channel.
- Modify `docs/superpowers/specs/2026-05-29-bluetooth-test-control-channel-design.md`
  - Ensures the Bluetooth channel documentation matches the `Serial3` implementation.

---

### Task 1: Runtime Loop Instrumentation

**Files:**
- Create: `SelfBalanceRobot/RuntimeStats.h`
- Create: `SelfBalanceRobot/RuntimeStats.cpp`
- Create: `tests/native/test_runtime_stats.cpp`
- Modify: `tests/native/Makefile`
- Modify: `SelfBalanceRobot/SelfBalanceRobot.ino`

- [ ] **Step 1: Write the failing runtime stats tests**

Create `tests/native/test_runtime_stats.cpp`:

```cpp
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
```

- [ ] **Step 2: Add the Makefile target and verify the test fails**

Modify `tests/native/Makefile`:

```make
.PHONY: test_runtime_stats

all: test_runtime_stats

$(BUILD_DIR)/test_runtime_stats: test_runtime_stats.cpp $(ROOT)/SelfBalanceRobot/RuntimeStats.cpp $(ROOT)/SelfBalanceRobot/RuntimeStats.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(ROOT)/SelfBalanceRobot test_runtime_stats.cpp $(ROOT)/SelfBalanceRobot/RuntimeStats.cpp -o $@

test_runtime_stats: $(BUILD_DIR)/test_runtime_stats
	./$(BUILD_DIR)/test_runtime_stats
```

Keep the existing `all:` dependencies and append `test_runtime_stats`; do not replace existing targets.

Run:

```bash
make -C tests/native test_runtime_stats
```

Expected: compile fails because `RuntimeStats.h` does not exist.

- [ ] **Step 3: Implement `RuntimeStats`**

Create `SelfBalanceRobot/RuntimeStats.h`:

```cpp
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
```

Create `SelfBalanceRobot/RuntimeStats.cpp`:

```cpp
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
```

- [ ] **Step 4: Run the new native test**

Run:

```bash
make -C tests/native test_runtime_stats
```

Expected: `test_runtime_stats PASS`.

- [ ] **Step 5: Wire runtime stats into the sketch**

Modify `SelfBalanceRobot/SelfBalanceRobot.ino`:

```cpp
#include "RuntimeStats.h"
```

Add a global near the other module globals:

```cpp
RuntimeStats runtimeStats;
```

At the start of the active balance-loop tick, after the early return:

```cpp
  const unsigned long workStartMicros = micros();
```

After each feedback refresh:

```cpp
  runtimeStats.recordFeedbackRefresh(true);
```

Wrap debug/telemetry printing:

```cpp
  const unsigned long telemetryStartMicros = micros();
  printDebug(lastFrame);
  const unsigned long telemetryMicros = micros() - telemetryStartMicros;
  if (telemetryMicros > 0) {
    runtimeStats.recordTelemetryPrint(telemetryMicros);
  }
```

At the end of the active tick:

```cpp
  runtimeStats.recordBalanceTick(
      elapsedMicros, micros() - workStartMicros, Config::BalanceLoopMicros);
```

In `printStatus`, append runtime counters:

```cpp
  const RuntimeStatsSnapshot runtime = runtimeStats.snapshot();
  out.print(F(" loopTicks="));
  out.print(runtime.balanceTicks);
  out.print(F(" workUs="));
  out.print(runtime.lastWorkMicros);
  out.print(F(" maxWorkUs="));
  out.print(runtime.maxWorkMicros);
  out.print(F(" missed="));
  out.print(runtime.missedDeadlines);
  out.print(F(" feedbackFull="));
  out.print(runtime.fullFeedbackRefreshes);
  out.print(F(" feedbackLight="));
  out.print(runtime.lightFeedbackRefreshes);
  out.print(F(" motorWrites="));
  out.print(runtime.motorWrites);
  out.print(F(" motorStops="));
  out.print(runtime.motorStops);
  out.print(F(" telemUs="));
  out.print(runtime.lastTelemetryMicros);
  out.print(F(" maxTelemUs="));
  out.print(runtime.maxTelemetryMicros);
```

- [ ] **Step 6: Verify and commit runtime instrumentation**

Run:

```bash
make -C tests/native all
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 SelfBalanceRobot
```

Expected:
- All native tests print `PASS`.
- Arduino compile succeeds.
- `STATUS` now includes `workUs=`, `maxWorkUs=`, `missed=`, `feedbackFull=`, `feedbackLight=`, `motorWrites=`, `motorStops=`, `telemUs=`, and `maxTelemUs=`.

Commit:

```bash
git add SelfBalanceRobot/RuntimeStats.h SelfBalanceRobot/RuntimeStats.cpp tests/native/test_runtime_stats.cpp tests/native/Makefile SelfBalanceRobot/SelfBalanceRobot.ino
git commit -m "feat: add runtime loop instrumentation"
```

---

### Task 2: Extract the Balance Pipeline

**Files:**
- Create: `SelfBalanceRobot/BalancePipeline.h`
- Create: `SelfBalanceRobot/BalancePipeline.cpp`
- Create: `tests/native/test_balance_pipeline.cpp`
- Modify: `tests/native/Makefile`
- Modify: `SelfBalanceRobot/SelfBalanceRobot.ino`

- [ ] **Step 1: Write the failing balance pipeline tests**

Create `tests/native/test_balance_pipeline.cpp`:

```cpp
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

static void test_persisted_session_uses_active_balance_point_without_manual_trim() {
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
```

- [ ] **Step 2: Add the Makefile target and verify the test fails**

Modify `tests/native/Makefile`:

```make
.PHONY: test_balance_pipeline

all: test_balance_pipeline

$(BUILD_DIR)/test_balance_pipeline: test_balance_pipeline.cpp $(ROOT)/SelfBalanceRobot/BalancePipeline.cpp $(ROOT)/SelfBalanceRobot/BalancePipeline.h $(ROOT)/SelfBalanceRobot/BalanceController.cpp $(ROOT)/SelfBalanceRobot/BalanceController.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(ROOT)/SelfBalanceRobot/config.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_balance_pipeline.cpp $(ROOT)/SelfBalanceRobot/BalancePipeline.cpp $(ROOT)/SelfBalanceRobot/BalanceController.cpp -o $@

test_balance_pipeline: $(BUILD_DIR)/test_balance_pipeline
	./$(BUILD_DIR)/test_balance_pipeline
```

Run:

```bash
make -C tests/native test_balance_pipeline
```

Expected: compile fails because `BalancePipeline.h` does not exist.

- [ ] **Step 3: Implement the balance pipeline interface**

Create `SelfBalanceRobot/BalancePipeline.h`:

```cpp
#ifndef BALANCE_PIPELINE_H
#define BALANCE_PIPELINE_H

#include "BalanceController.h"
#include "RobotTypes.h"
#include "config.h"

struct BalancePipelineInput {
  SensorFrame frame;
  WheelFeedback wheelFeedback;
  float uprightAngleDegrees = 0.0f;
  float activeBalancePointDegrees = Config::AutoArmDefaultBalancePointDegrees;
  float currentTrimDegrees = Config::BalanceAngleTrimDegrees;
  unsigned long balancingStartMillis = 0;
  bool balanceSessionUsesPersistedPoint = false;
  float dtSeconds = 0.0f;
};

struct BalancePipelineOutput {
  float baseTargetDegrees = 0.0f;
  float targetAngleDegrees = 0.0f;
  float travelHoldTargetCorrectionDegrees = 0.0f;
  int16_t rawBalanceOutput = 0;
  int16_t balanceOutput = 0;
  MotorCommand motorCommand;
};

class BalancePipeline {
public:
  BalancePipelineOutput update(const BalancePipelineInput& input,
                               BalanceController& controller) const;

private:
  float baseBalanceTargetDegrees(const BalancePipelineInput& input) const;
  float rampedTargetDegrees(const BalancePipelineInput& input,
                            float finalTargetDegrees) const;
  float clampWheelSpeedTargetCorrection(float correctionDegrees) const;
  float clampTravelHoldTargetCorrection(float correctionDegrees) const;
  int16_t clampMotorCommand(float command) const;
  int16_t applyLargeLeanBoost(int16_t balanceOutput, float angleError) const;
  int16_t applyMinimumBalanceCommand(int16_t balanceOutput,
                                     float angleError) const;
  int16_t applyWheelSpeedDamping(int16_t balanceOutput, float angleError,
                                 const WheelFeedback& wheelFeedback) const;
};

#endif
```

- [ ] **Step 4: Implement the balance pipeline behavior**

Create `SelfBalanceRobot/BalancePipeline.cpp` by moving the equivalent control-path helpers out of `SelfBalanceRobot.ino`:

```cpp
#include "BalancePipeline.h"

BalancePipelineOutput
BalancePipeline::update(const BalancePipelineInput& input,
                        BalanceController& controller) const {
  BalancePipelineOutput output;

  const float speedCorrection = clampWheelSpeedTargetCorrection(
      input.wheelFeedback.averageSpeedRpm *
      Config::WheelSpeedTargetCorrectionDegreesPerRpm);
  output.travelHoldTargetCorrectionDegrees =
      clampTravelHoldTargetCorrection(
          input.wheelFeedback.averagePositionDegrees *
          Config::TravelHoldTargetDegreesPerWheelDegree);
  output.baseTargetDegrees = baseBalanceTargetDegrees(input);
  const float finalTarget = output.baseTargetDegrees + speedCorrection +
                            output.travelHoldTargetCorrectionDegrees;
  output.targetAngleDegrees = rampedTargetDegrees(input, finalTarget);

  controller.setTargetAngle(output.targetAngleDegrees);
  output.rawBalanceOutput =
      controller.update(input.frame.angleDegrees,
                        input.frame.angleRateDegPerSec,
                        input.dtSeconds);

  const float angleError =
      output.targetAngleDegrees - input.frame.angleDegrees;
  int16_t shapedOutput =
      applyMinimumBalanceCommand(output.rawBalanceOutput, angleError);
  shapedOutput = applyLargeLeanBoost(shapedOutput, angleError);
  shapedOutput =
      applyWheelSpeedDamping(shapedOutput, angleError, input.wheelFeedback);

  output.balanceOutput = shapedOutput;
  output.motorCommand.left = shapedOutput;
  output.motorCommand.right = shapedOutput;
  return output;
}

float BalancePipeline::baseBalanceTargetDegrees(
    const BalancePipelineInput& input) const {
  if (input.balanceSessionUsesPersistedPoint) {
    return input.activeBalancePointDegrees;
  }
  return input.uprightAngleDegrees + input.currentTrimDegrees;
}

float BalancePipeline::rampedTargetDegrees(
    const BalancePipelineInput& input, float finalTargetDegrees) const {
  if (input.balanceSessionUsesPersistedPoint) {
    return finalTargetDegrees;
  }

  const unsigned long elapsed =
      input.frame.nowMillis - input.balancingStartMillis;
  const unsigned long rampMs = Config::BalanceTargetRampMillis;
  if (elapsed >= rampMs) {
    return finalTargetDegrees;
  }

  const float rampFraction =
      static_cast<float>(elapsed) / static_cast<float>(rampMs);
  return input.uprightAngleDegrees +
         rampFraction * (finalTargetDegrees - input.uprightAngleDegrees);
}

float BalancePipeline::clampWheelSpeedTargetCorrection(
    float correctionDegrees) const {
  const float limit = Config::MaxWheelSpeedTargetCorrectionDegrees;
  if (correctionDegrees > limit) return limit;
  if (correctionDegrees < -limit) return -limit;
  return correctionDegrees;
}

float BalancePipeline::clampTravelHoldTargetCorrection(
    float correctionDegrees) const {
  const float limit = Config::MaxTravelHoldTargetCorrectionDegrees;
  if (correctionDegrees > limit) return limit;
  if (correctionDegrees < -limit) return -limit;
  return correctionDegrees;
}

int16_t BalancePipeline::clampMotorCommand(float command) const {
  if (command > Config::MaxMotorCommand) return Config::MaxMotorCommand;
  if (command < -Config::MaxMotorCommand) return -Config::MaxMotorCommand;
  return static_cast<int16_t>(command);
}

int16_t BalancePipeline::applyLargeLeanBoost(int16_t balanceOutput,
                                             float angleError) const {
  const float absAngleError = angleError < 0.0f ? -angleError : angleError;
  if (absAngleError <= Config::LargeLeanBoostAngleDegrees) {
    return balanceOutput;
  }

  const float extra =
      (absAngleError - Config::LargeLeanBoostAngleDegrees) *
      Config::LargeLeanBoostCommandPerDegree;
  const float correctionDirection = angleError < 0.0f ? 1.0f : -1.0f;
  return clampMotorCommand(static_cast<float>(balanceOutput) +
                           correctionDirection * extra);
}

int16_t BalancePipeline::applyMinimumBalanceCommand(
    int16_t balanceOutput, float angleError) const {
  const float absAngleError = angleError < 0.0f ? -angleError : angleError;
  if (absAngleError < Config::MinBalanceBoostAngleDegrees) {
    return balanceOutput;
  }

  const int16_t minimum = Config::MinBalanceMotorCommand;
  if (minimum <= 0) {
    return balanceOutput;
  }

  if (balanceOutput > 0 && balanceOutput < minimum) return minimum;
  if (balanceOutput < 0 && balanceOutput > -minimum) {
    return static_cast<int16_t>(-minimum);
  }
  return balanceOutput;
}

int16_t BalancePipeline::applyWheelSpeedDamping(
    int16_t balanceOutput, float angleError,
    const WheelFeedback& wheelFeedback) const {
  const float absAngleError = angleError < 0.0f ? -angleError : angleError;
  if (absAngleError > Config::WheelSpeedDampingMaxAngleErrorDegrees) {
    return balanceOutput;
  }

  return clampMotorCommand(
      static_cast<float>(balanceOutput) -
      wheelFeedback.averageSpeedRpm *
          Config::WheelSpeedDampingCommandPerRpm);
}
```

- [ ] **Step 5: Run the new pipeline tests**

Run:

```bash
make -C tests/native test_balance_pipeline
```

Expected: `test_balance_pipeline PASS`.

- [ ] **Step 6: Replace inline balance math in the sketch**

Modify `SelfBalanceRobot/SelfBalanceRobot.ino`:

```cpp
#include "BalancePipeline.h"
```

Add a global:

```cpp
BalancePipeline balancePipeline;
```

Replace the balance branch body that computes speed correction, ramp target, PID output, boosts, damping, and `lastMotorOutput` with:

```cpp
    BalancePipelineInput pipelineInput;
    pipelineInput.frame = frame;
    pipelineInput.wheelFeedback = lastWheelFeedback;
    pipelineInput.uprightAngleDegrees = robotState.uprightAngleDegrees();
    pipelineInput.activeBalancePointDegrees = activeBalancePointDegrees;
    pipelineInput.currentTrimDegrees = currentTrimDegrees;
    pipelineInput.balancingStartMillis = balancingStartMillis;
    pipelineInput.balanceSessionUsesPersistedPoint =
        balanceSessionUsesPersistedPoint;
    pipelineInput.dtSeconds = dtSeconds;

    const BalancePipelineOutput pipelineOutput =
        balancePipeline.update(pipelineInput, balance);
    lastTargetAngle = pipelineOutput.targetAngleDegrees;
    lastRawBalanceOutput = pipelineOutput.rawBalanceOutput;
    lastBalanceOutput = pipelineOutput.balanceOutput;
    lastTravelHoldTargetCorrection =
        pipelineOutput.travelHoldTargetCorrectionDegrees;
    updateBalancePointLearning(frame, pipelineOutput.baseTargetDegrees,
                               pipelineOutput.balanceOutput, nowMillis);
    lastMotorOutput = pipelineOutput.motorCommand;
    motors.write(lastMotorOutput);
    runtimeStats.recordMotorWrite();
```

Remove these now-duplicated helper declarations and definitions from `SelfBalanceRobot.ino`:

```cpp
float baseBalanceTargetDegrees();
float clampWheelSpeedTargetCorrection(float correctionDegrees);
float clampTravelHoldTargetCorrection(float correctionDegrees);
int16_t clampMotorCommand(float command);
int16_t applyLargeLeanBoost(int16_t balanceOutput, float angleError);
int16_t applyMinimumBalanceCommand(int16_t balanceOutput, float angleError);
```

- [ ] **Step 7: Verify and commit the balance extraction**

Run:

```bash
make -C tests/native all
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 SelfBalanceRobot
```

Expected:
- All native tests print `PASS`.
- Arduino compile succeeds.
- The sketch still sends identical motor command signs for the same sensor frame and wheel feedback.

Commit:

```bash
git add SelfBalanceRobot/BalancePipeline.h SelfBalanceRobot/BalancePipeline.cpp tests/native/test_balance_pipeline.cpp tests/native/Makefile SelfBalanceRobot/SelfBalanceRobot.ino
git commit -m "refactor: extract balance pipeline"
```

---

### Task 3: Gate Repeated Motor Output Writes

**Files:**
- Create: `SelfBalanceRobot/MotorOutputLatch.h`
- Create: `SelfBalanceRobot/MotorOutputLatch.cpp`
- Create: `tests/native/test_motor_output_latch.cpp`
- Modify: `tests/native/Makefile`
- Modify: `SelfBalanceRobot/SelfBalanceRobot.ino`

- [ ] **Step 1: Write the failing motor output latch tests**

Create `tests/native/test_motor_output_latch.cpp`:

```cpp
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
```

- [ ] **Step 2: Add the Makefile target and verify the test fails**

Modify `tests/native/Makefile`:

```make
.PHONY: test_motor_output_latch

all: test_motor_output_latch

$(BUILD_DIR)/test_motor_output_latch: test_motor_output_latch.cpp $(ROOT)/SelfBalanceRobot/MotorOutputLatch.cpp $(ROOT)/SelfBalanceRobot/MotorOutputLatch.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_motor_output_latch.cpp $(ROOT)/SelfBalanceRobot/MotorOutputLatch.cpp -o $@

test_motor_output_latch: $(BUILD_DIR)/test_motor_output_latch
	./$(BUILD_DIR)/test_motor_output_latch
```

Run:

```bash
make -C tests/native test_motor_output_latch
```

Expected: compile fails because `MotorOutputLatch.h` does not exist.

- [ ] **Step 3: Implement `MotorOutputLatch`**

Create `SelfBalanceRobot/MotorOutputLatch.h`:

```cpp
#ifndef MOTOR_OUTPUT_LATCH_H
#define MOTOR_OUTPUT_LATCH_H

#include "RobotTypes.h"

class MotorOutputLatch {
public:
  bool shouldWrite(const MotorCommand& command);
  bool shouldStop();
  void reset();

private:
  MotorCommand lastCommand_;
  bool hasCommand_ = false;
  bool stopped_ = true;
};

#endif
```

Create `SelfBalanceRobot/MotorOutputLatch.cpp`:

```cpp
#include "MotorOutputLatch.h"

bool MotorOutputLatch::shouldWrite(const MotorCommand& command) {
  if (hasCommand_ && lastCommand_.left == command.left &&
      lastCommand_.right == command.right && !stopped_) {
    return false;
  }

  lastCommand_ = command;
  hasCommand_ = true;
  stopped_ = false;
  return true;
}

bool MotorOutputLatch::shouldStop() {
  if (stopped_) {
    return false;
  }

  lastCommand_ = MotorCommand();
  hasCommand_ = true;
  stopped_ = true;
  return true;
}

void MotorOutputLatch::reset() {
  lastCommand_ = MotorCommand();
  hasCommand_ = false;
  stopped_ = false;
}
```

- [ ] **Step 4: Run the motor latch tests**

Run:

```bash
make -C tests/native test_motor_output_latch
```

Expected: `test_motor_output_latch PASS`.

- [ ] **Step 5: Use the latch in the sketch**

Modify `SelfBalanceRobot/SelfBalanceRobot.ino`:

```cpp
#include "MotorOutputLatch.h"
```

Add a global:

```cpp
MotorOutputLatch motorOutputLatch;
```

When entering balancing, reset the latch so the first balance command is written:

```cpp
    motorOutputLatch.reset();
```

Replace direct writes in the balancing branch:

```cpp
    if (motorOutputLatch.shouldWrite(lastMotorOutput)) {
      motors.write(lastMotorOutput);
      runtimeStats.recordMotorWrite();
    }
```

Replace the motor test write:

```cpp
    if (motorOutputLatch.shouldWrite(lastMotorOutput)) {
      motors.write(lastMotorOutput);
      runtimeStats.recordMotorWrite();
    }
```

Replace repeated idle stop calls:

```cpp
    if (motorOutputLatch.shouldStop()) {
      motors.stop();
      runtimeStats.recordMotorStop();
    }
```

In `applyStopCommand`, after setting `lastMotorOutput = MotorCommand();`, force an immediate hardware stop and align the latch:

```cpp
  motors.stop();
  runtimeStats.recordMotorStop();
  motorOutputLatch.reset();
```

- [ ] **Step 6: Verify and commit motor write gating**

Run:

```bash
make -C tests/native all
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 SelfBalanceRobot
```

Expected:
- All native tests print `PASS`.
- Arduino compile succeeds.
- `STATUS` shows `motorWrites=` growing more slowly when output is steady.
- `STOP` still stops motors immediately.

Commit:

```bash
git add SelfBalanceRobot/MotorOutputLatch.h SelfBalanceRobot/MotorOutputLatch.cpp tests/native/test_motor_output_latch.cpp tests/native/Makefile SelfBalanceRobot/SelfBalanceRobot.ino
git commit -m "perf: avoid repeated identical motor writes"
```

---

### Task 4: Decimate Optional Encoder Speed Refresh

**Files:**
- Create: `SelfBalanceRobot/FeedbackPolicy.h`
- Create: `SelfBalanceRobot/FeedbackPolicy.cpp`
- Create: `tests/native/test_feedback_policy.cpp`
- Modify: `SelfBalanceRobot/Motors.h`
- Modify: `SelfBalanceRobot/Motors.cpp`
- Modify: `SelfBalanceRobot/config.h`
- Modify: `SelfBalanceRobot/SelfBalanceRobot.ino`
- Modify: `tests/native/Makefile`

- [ ] **Step 1: Write the failing feedback policy tests**

Create `tests/native/test_feedback_policy.cpp`:

```cpp
#include <cassert>
#include <cstdlib>
#include <iostream>

#include "../../SelfBalanceRobot/FeedbackPolicy.h"

static void test_speed_features_force_full_refresh_every_tick() {
  FeedbackPolicy policy;
  policy.configure(5);

  FeedbackRequest request;
  request.speedTargetCorrectionEnabled = true;
  request.speedDampingEnabled = false;
  request.forceFullRefresh = false;

  assert(policy.nextMode(request) == MotorFeedbackMode::Full);
  assert(policy.nextMode(request) == MotorFeedbackMode::Full);
}

static void test_disabled_speed_features_use_periodic_full_refresh() {
  FeedbackPolicy policy;
  policy.configure(3);

  FeedbackRequest request;
  request.speedTargetCorrectionEnabled = false;
  request.speedDampingEnabled = false;
  request.forceFullRefresh = false;

  assert(policy.nextMode(request) == MotorFeedbackMode::Full);
  assert(policy.nextMode(request) == MotorFeedbackMode::PositionAndPwm);
  assert(policy.nextMode(request) == MotorFeedbackMode::PositionAndPwm);
  assert(policy.nextMode(request) == MotorFeedbackMode::Full);
}

static void test_forced_refresh_does_not_break_periodic_count() {
  FeedbackPolicy policy;
  policy.configure(4);

  FeedbackRequest request;
  request.speedTargetCorrectionEnabled = false;
  request.speedDampingEnabled = false;
  request.forceFullRefresh = false;

  assert(policy.nextMode(request) == MotorFeedbackMode::Full);
  assert(policy.nextMode(request) == MotorFeedbackMode::PositionAndPwm);
  request.forceFullRefresh = true;
  assert(policy.nextMode(request) == MotorFeedbackMode::Full);
  request.forceFullRefresh = false;
  assert(policy.nextMode(request) == MotorFeedbackMode::PositionAndPwm);
  assert(policy.nextMode(request) == MotorFeedbackMode::Full);
}

int main() {
  test_speed_features_force_full_refresh_every_tick();
  test_disabled_speed_features_use_periodic_full_refresh();
  test_forced_refresh_does_not_break_periodic_count();

  std::cout << "test_feedback_policy PASS\n";
  return EXIT_SUCCESS;
}
```

- [ ] **Step 2: Add the Makefile target and verify the test fails**

Modify `tests/native/Makefile`:

```make
.PHONY: test_feedback_policy

all: test_feedback_policy

$(BUILD_DIR)/test_feedback_policy: test_feedback_policy.cpp $(ROOT)/SelfBalanceRobot/FeedbackPolicy.cpp $(ROOT)/SelfBalanceRobot/FeedbackPolicy.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_feedback_policy.cpp $(ROOT)/SelfBalanceRobot/FeedbackPolicy.cpp -o $@

test_feedback_policy: $(BUILD_DIR)/test_feedback_policy
	./$(BUILD_DIR)/test_feedback_policy
```

Run:

```bash
make -C tests/native test_feedback_policy
```

Expected: compile fails because `FeedbackPolicy.h` does not exist.

- [ ] **Step 3: Implement feedback policy types**

Create `SelfBalanceRobot/FeedbackPolicy.h`:

```cpp
#ifndef FEEDBACK_POLICY_H
#define FEEDBACK_POLICY_H

#include <stdint.h>

enum class MotorFeedbackMode : uint8_t {
  PositionAndPwm,
  Full
};

struct FeedbackRequest {
  bool speedTargetCorrectionEnabled = false;
  bool speedDampingEnabled = false;
  bool forceFullRefresh = false;
};

class FeedbackPolicy {
public:
  void configure(uint8_t fullRefreshPeriodTicks);
  MotorFeedbackMode nextMode(const FeedbackRequest& request);

private:
  uint8_t fullRefreshPeriodTicks_ = 1;
  uint8_t ticksSinceFullRefresh_ = 0;
};

#endif
```

Create `SelfBalanceRobot/FeedbackPolicy.cpp`:

```cpp
#include "FeedbackPolicy.h"

void FeedbackPolicy::configure(uint8_t fullRefreshPeriodTicks) {
  fullRefreshPeriodTicks_ = fullRefreshPeriodTicks == 0
                                ? 1
                                : fullRefreshPeriodTicks;
  ticksSinceFullRefresh_ = fullRefreshPeriodTicks_;
}

MotorFeedbackMode FeedbackPolicy::nextMode(const FeedbackRequest& request) {
  const bool speedFeatureEnabled =
      request.speedTargetCorrectionEnabled || request.speedDampingEnabled;
  if (speedFeatureEnabled || request.forceFullRefresh ||
      ticksSinceFullRefresh_ >= fullRefreshPeriodTicks_) {
    ticksSinceFullRefresh_ = 0;
    return MotorFeedbackMode::Full;
  }

  ++ticksSinceFullRefresh_;
  return MotorFeedbackMode::PositionAndPwm;
}
```

- [ ] **Step 4: Run the feedback policy tests**

Run:

```bash
make -C tests/native test_feedback_policy
```

Expected: `test_feedback_policy PASS`.

- [ ] **Step 5: Add feedback mode support to `Motors`**

Modify `SelfBalanceRobot/Motors.h`:

```cpp
#include "FeedbackPolicy.h"
```

Change the feedback method declaration:

```cpp
  WheelFeedback updateFeedback(
      MotorFeedbackMode mode = MotorFeedbackMode::Full);
```

Modify `SelfBalanceRobot/Motors.cpp`:

```cpp
WheelFeedback Motors::updateFeedback(MotorFeedbackMode mode) {
  if (mode == MotorFeedbackMode::Full) {
    rightMotor_.updateSpeed();
    leftMotor_.updateSpeed();
    feedback_.rightSpeedRpm = -rightMotor_.getCurrentSpeed();
    feedback_.leftSpeedRpm = leftMotor_.getCurrentSpeed();
    feedback_.averageSpeedRpm =
        (feedback_.rightSpeedRpm + feedback_.leftSpeedRpm) * 0.5f;
  }

  rightMotor_.updateCurPos();
  leftMotor_.updateCurPos();

  feedback_.rightPositionDegrees = -rightMotor_.getCurPos();
  feedback_.leftPositionDegrees = leftMotor_.getCurPos();
  feedback_.averagePositionDegrees =
      (static_cast<float>(feedback_.rightPositionDegrees) +
       static_cast<float>(feedback_.leftPositionDegrees)) *
      0.5f;
  feedback_.rightPwm = -rightMotor_.getCurPwm();
  feedback_.leftPwm = leftMotor_.getCurPwm();
  return feedback_;
}
```

- [ ] **Step 6: Configure feedback decimation**

Modify `SelfBalanceRobot/config.h`:

```cpp
  constexpr uint8_t FeedbackFullRefreshPeriodTicks = 5;
```

Modify `SelfBalanceRobot/SelfBalanceRobot.ino`:

```cpp
#include "FeedbackPolicy.h"
```

Add a global:

```cpp
FeedbackPolicy feedbackPolicy;
```

In `setup()` after configuring other modules:

```cpp
  feedbackPolicy.configure(Config::FeedbackFullRefreshPeriodTicks);
```

Replace the unconditional feedback update:

```cpp
  FeedbackRequest feedbackRequest;
  feedbackRequest.speedTargetCorrectionEnabled =
      Config::WheelSpeedTargetCorrectionDegreesPerRpm != 0.0f ||
      Config::MaxWheelSpeedTargetCorrectionDegrees != 0.0f;
  feedbackRequest.speedDampingEnabled =
      Config::WheelSpeedDampingCommandPerRpm != 0.0f;
  feedbackRequest.forceFullRefresh = bluetoothTelemetryEnabled;
  const MotorFeedbackMode feedbackMode =
      feedbackPolicy.nextMode(feedbackRequest);
  lastWheelFeedback = motors.updateFeedback(feedbackMode);
  runtimeStats.recordFeedbackRefresh(feedbackMode == MotorFeedbackMode::Full);
```

Keep the immediate full refresh after `motors.resetTravel()`:

```cpp
    lastWheelFeedback = motors.updateFeedback(MotorFeedbackMode::Full);
    runtimeStats.recordFeedbackRefresh(true);
```

- [ ] **Step 7: Verify and commit feedback decimation**

Run:

```bash
make -C tests/native all
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 SelfBalanceRobot
```

Expected:
- All native tests print `PASS`.
- Arduino compile succeeds.
- With wheel speed constants still set to `0.0f`, `STATUS` shows both `feedbackFull=` and `feedbackLight=` increasing.
- If either speed correction or damping is set nonzero, `feedbackFull=` increases every balance tick.

Commit:

```bash
git add SelfBalanceRobot/FeedbackPolicy.h SelfBalanceRobot/FeedbackPolicy.cpp SelfBalanceRobot/Motors.h SelfBalanceRobot/Motors.cpp SelfBalanceRobot/config.h SelfBalanceRobot/SelfBalanceRobot.ino tests/native/test_feedback_policy.cpp tests/native/Makefile
git commit -m "perf: decimate optional encoder speed refresh"
```

---

### Task 5: Centralize Telemetry Formatting

**Files:**
- Create: `SelfBalanceRobot/TelemetryFormatter.h`
- Create: `tests/native/test_telemetry_formatter.cpp`
- Modify: `tests/native/Makefile`
- Modify: `SelfBalanceRobot/SelfBalanceRobot.ino`

- [ ] **Step 1: Write the failing telemetry formatter tests**

Create `tests/native/test_telemetry_formatter.cpp`:

```cpp
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

#include "../../SelfBalanceRobot/TelemetryFormatter.h"

class CaptureOutput {
public:
  void print(const char* text) { buffer_ << text; }
  void print(char text) { buffer_ << text; }
  void print(int value) { buffer_ << value; }
  void print(uint32_t value) { buffer_ << value; }
  void print(unsigned long value) { buffer_ << value; }
  void print(float value) { buffer_ << value; }
  void println(const char* text) { buffer_ << text << '\n'; }
  void println(float value) { buffer_ << value << '\n'; }
  void println(int value) { buffer_ << value << '\n'; }
  void println() { buffer_ << '\n'; }
  std::string str() const { return buffer_.str(); }

private:
  std::ostringstream buffer_;
};

static TelemetrySnapshot sample() {
  TelemetrySnapshot snapshot;
  snapshot.mode = RobotMode::Balancing;
  snapshot.angleDegrees = -1.25f;
  snapshot.targetAngleDegrees = 0.75f;
  snapshot.trimDegrees = -2.3f;
  snapshot.activeBalancePointDegrees = 0.75f;
  snapshot.storedBalancePoint = true;
  snapshot.autoArmEnabled = true;
  snapshot.autoAngleErrorDegrees = 0.1f;
  snapshot.gyroFresh = true;
  snapshot.gyroRateDegPerSec = 2.0f;
  snapshot.filteredRateDegPerSec = 1.5f;
  snapshot.rawBalanceOutput = -10;
  snapshot.balanceOutput = -16;
  snapshot.averageSpeedRpm = 0.0f;
  snapshot.averagePositionDegrees = 5.0f;
  snapshot.travelHoldTargetCorrectionDegrees = 0.01f;
  snapshot.leftMotor = -16;
  snapshot.rightMotor = -16;
  snapshot.leftPwm = -16;
  snapshot.rightPwm = -16;
  snapshot.kp = 42.0f;
  snapshot.ki = 0.0f;
  snapshot.kd = 1.0f;
  snapshot.runtime.balanceTicks = 12;
  snapshot.runtime.lastWorkMicros = 3000;
  snapshot.runtime.maxWorkMicros = 4200;
  return snapshot;
}

static void test_status_contains_runtime_and_control_fields() {
  CaptureOutput out;

  TelemetryFormatter::printStatus(out, sample());
  const std::string text = out.str();

  assert(text.find("status mode=balancing") != std::string::npos);
  assert(text.find(" angle=") != std::string::npos);
  assert(text.find(" target=") != std::string::npos);
  assert(text.find(" raw=") != std::string::npos);
  assert(text.find(" balance=") != std::string::npos);
  assert(text.find(" loopTicks=") != std::string::npos);
  assert(text.find(" workUs=") != std::string::npos);
}

static void test_debug_uses_shorter_field_set_than_status() {
  CaptureOutput out;

  TelemetryFormatter::printDebug(out, sample());
  const std::string text = out.str();

  assert(text.find("mode=balancing") != std::string::npos);
  assert(text.find("loopUs=") != std::string::npos);
  assert(text.find("resetRaw=") == std::string::npos);
  assert(text.find("feedbackFull=") == std::string::npos);
}

static void test_bluetooth_telemetry_keeps_telem_prefix() {
  CaptureOutput out;

  TelemetryFormatter::printBluetoothTelemetry(out, sample());
  const std::string text = out.str();

  assert(text.find("telem mode=balancing") == 0);
  assert(text.find(" auto=") != std::string::npos);
  assert(text.find(" gyroFresh=") != std::string::npos);
}

int main() {
  test_status_contains_runtime_and_control_fields();
  test_debug_uses_shorter_field_set_than_status();
  test_bluetooth_telemetry_keeps_telem_prefix();

  std::cout << "test_telemetry_formatter PASS\n";
  return EXIT_SUCCESS;
}
```

- [ ] **Step 2: Add the Makefile target and verify the test fails**

Modify `tests/native/Makefile`:

```make
.PHONY: test_telemetry_formatter

all: test_telemetry_formatter

$(BUILD_DIR)/test_telemetry_formatter: test_telemetry_formatter.cpp $(ROOT)/SelfBalanceRobot/TelemetryFormatter.h $(ROOT)/SelfBalanceRobot/RuntimeStats.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_telemetry_formatter.cpp -o $@

test_telemetry_formatter: $(BUILD_DIR)/test_telemetry_formatter
	./$(BUILD_DIR)/test_telemetry_formatter
```

Run:

```bash
make -C tests/native test_telemetry_formatter
```

Expected: compile fails because `TelemetryFormatter.h` does not exist.

- [ ] **Step 3: Implement `TelemetryFormatter`**

Create `SelfBalanceRobot/TelemetryFormatter.h`:

```cpp
#ifndef TELEMETRY_FORMATTER_H
#define TELEMETRY_FORMATTER_H

#include "RobotTypes.h"
#include "RuntimeStats.h"

struct TelemetrySnapshot {
  RobotMode mode = RobotMode::Disarmed;
  unsigned long loopMicros = 0;
  float angleDegrees = 0.0f;
  float uprightAngleDegrees = 0.0f;
  float targetAngleDegrees = 0.0f;
  float trimDegrees = 0.0f;
  float activeBalancePointDegrees = 0.0f;
  bool storedBalancePoint = false;
  uint8_t resetRaw = 0;
  bool autoArmEnabled = false;
  float autoAngleErrorDegrees = 0.0f;
  bool gyroFresh = false;
  float gyroRateDegPerSec = 0.0f;
  float filteredRateDegPerSec = 0.0f;
  int16_t rawBalanceOutput = 0;
  int16_t balanceOutput = 0;
  float averageSpeedRpm = 0.0f;
  float averagePositionDegrees = 0.0f;
  float travelHoldTargetCorrectionDegrees = 0.0f;
  int16_t leftMotor = 0;
  int16_t rightMotor = 0;
  int16_t leftPwm = 0;
  int16_t rightPwm = 0;
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  RuntimeStatsSnapshot runtime;
};

class TelemetryFormatter {
public:
  template <typename Out>
  static void printMode(Out& out, RobotMode mode) {
    switch (mode) {
    case RobotMode::Disarmed: out.print("disarmed"); break;
    case RobotMode::Calibrating: out.print("calibrating"); break;
    case RobotMode::Balancing: out.print("balancing"); break;
    case RobotMode::Fault: out.print("fault"); break;
    }
  }

  template <typename Out>
  static void printStatus(Out& out, const TelemetrySnapshot& snapshot) {
    out.print("status mode=");
    printMode(out, snapshot.mode);
    printCommonControlFields(out, snapshot);
    out.print(" stored=");
    out.print(snapshot.storedBalancePoint ? "yes" : "no");
    out.print(" resetRaw=0x");
    out.print(snapshot.resetRaw);
    out.print(" ki=");
    out.print(snapshot.ki);
    out.print(" loopTicks=");
    out.print(snapshot.runtime.balanceTicks);
    out.print(" workUs=");
    out.print(snapshot.runtime.lastWorkMicros);
    out.print(" maxWorkUs=");
    out.print(snapshot.runtime.maxWorkMicros);
    out.print(" missed=");
    out.print(snapshot.runtime.missedDeadlines);
    out.print(" feedbackFull=");
    out.print(snapshot.runtime.fullFeedbackRefreshes);
    out.print(" feedbackLight=");
    out.print(snapshot.runtime.lightFeedbackRefreshes);
    out.print(" motorWrites=");
    out.print(snapshot.runtime.motorWrites);
    out.print(" motorStops=");
    out.print(snapshot.runtime.motorStops);
    out.print(" telemUs=");
    out.print(snapshot.runtime.lastTelemetryMicros);
    out.print(" maxTelemUs=");
    out.println(snapshot.runtime.maxTelemetryMicros);
  }

  template <typename Out>
  static void printDebug(Out& out, const TelemetrySnapshot& snapshot) {
    out.print("mode=");
    printMode(out, snapshot.mode);
    out.print(" loopUs=");
    out.print(snapshot.loopMicros);
    printCommonControlFields(out, snapshot);
    out.print(" lpwm=");
    out.print(snapshot.leftPwm);
    out.print(" rpwm=");
    out.print(snapshot.rightPwm);
    out.print(" kd=");
    out.println(snapshot.kd);
  }

  template <typename Out>
  static void printBluetoothTelemetry(Out& out,
                                      const TelemetrySnapshot& snapshot) {
    out.print("telem mode=");
    printMode(out, snapshot.mode);
    out.print(" angle=");
    out.print(snapshot.angleDegrees);
    out.print(" target=");
    out.print(snapshot.targetAngleDegrees);
    out.print(" auto=");
    out.print(snapshot.autoArmEnabled ? "on" : "off");
    out.print(" autoErr=");
    out.print(snapshot.autoAngleErrorDegrees);
    out.print(" gyroFresh=");
    out.print(snapshot.gyroFresh ? "yes" : "no");
    out.print(" gyroRate=");
    out.print(snapshot.gyroRateDegPerSec);
    out.print(" rate=");
    out.print(snapshot.filteredRateDegPerSec);
    out.print(" balance=");
    out.print(snapshot.balanceOutput);
    out.print(" left=");
    out.print(snapshot.leftMotor);
    out.print(" right=");
    out.println(snapshot.rightMotor);
  }

private:
  template <typename Out>
  static void printCommonControlFields(Out& out,
                                       const TelemetrySnapshot& snapshot) {
    out.print(" angle=");
    out.print(snapshot.angleDegrees);
    out.print(" upright=");
    out.print(snapshot.uprightAngleDegrees);
    out.print(" trim=");
    out.print(snapshot.trimDegrees);
    out.print(" target=");
    out.print(snapshot.targetAngleDegrees);
    out.print(" bp=");
    out.print(snapshot.activeBalancePointDegrees);
    out.print(" auto=");
    out.print(snapshot.autoArmEnabled ? "on" : "off");
    out.print(" autoErr=");
    out.print(snapshot.autoAngleErrorDegrees);
    out.print(" gyroFresh=");
    out.print(snapshot.gyroFresh ? "yes" : "no");
    out.print(" gyroRate=");
    out.print(snapshot.gyroRateDegPerSec);
    out.print(" rate=");
    out.print(snapshot.filteredRateDegPerSec);
    out.print(" raw=");
    out.print(snapshot.rawBalanceOutput);
    out.print(" balance=");
    out.print(snapshot.balanceOutput);
    out.print(" speed=");
    out.print(snapshot.averageSpeedRpm);
    out.print(" pos=");
    out.print(snapshot.averagePositionDegrees);
    out.print(" hold=");
    out.print(snapshot.travelHoldTargetCorrectionDegrees);
    out.print(" left=");
    out.print(snapshot.leftMotor);
    out.print(" right=");
    out.print(snapshot.rightMotor);
    out.print(" kp=");
    out.print(snapshot.kp);
  }
};

#endif
```

- [ ] **Step 4: Run the formatter tests**

Run:

```bash
make -C tests/native test_telemetry_formatter
```

Expected: `test_telemetry_formatter PASS`.

- [ ] **Step 5: Build a snapshot in the sketch and replace duplicate printers**

Modify `SelfBalanceRobot/SelfBalanceRobot.ino`:

```cpp
#include "TelemetryFormatter.h"
```

Add a helper declaration:

```cpp
TelemetrySnapshot buildTelemetrySnapshot(const SensorFrame& frame);
```

Add the helper:

```cpp
TelemetrySnapshot buildTelemetrySnapshot(const SensorFrame& frame) {
  TelemetrySnapshot snapshot;
  snapshot.mode = robotState.mode();
  snapshot.loopMicros = lastLoopMicros;
  snapshot.angleDegrees = frame.angleDegrees;
  snapshot.uprightAngleDegrees = robotState.uprightAngleDegrees();
  snapshot.targetAngleDegrees = lastTargetAngle;
  snapshot.trimDegrees = currentTrimDegrees;
  snapshot.activeBalancePointDegrees = activeBalancePointDegrees;
  snapshot.storedBalancePoint = balancePointStore.hasStoredBalancePoint();
  snapshot.resetRaw = resetCauseRaw;
  snapshot.autoArmEnabled = runtimeAutoArmEnabled;
  snapshot.autoAngleErrorDegrees = autoArm.angleErrorDegrees(frame);
  snapshot.gyroFresh = frame.gyroFresh;
  snapshot.gyroRateDegPerSec = frame.angleRateDegPerSec;
  snapshot.filteredRateDegPerSec =
      balance.lastMeasuredAngleRateDegreesPerSecond();
  snapshot.rawBalanceOutput = lastRawBalanceOutput;
  snapshot.balanceOutput = lastBalanceOutput;
  snapshot.averageSpeedRpm = lastWheelFeedback.averageSpeedRpm;
  snapshot.averagePositionDegrees = lastWheelFeedback.averagePositionDegrees;
  snapshot.travelHoldTargetCorrectionDegrees =
      lastTravelHoldTargetCorrection;
  snapshot.leftMotor = lastMotorOutput.left;
  snapshot.rightMotor = lastMotorOutput.right;
  snapshot.leftPwm = lastWheelFeedback.leftPwm;
  snapshot.rightPwm = lastWheelFeedback.rightPwm;
  snapshot.kp = currentKp;
  snapshot.ki = currentKi;
  snapshot.kd = currentKd;
  snapshot.runtime = runtimeStats.snapshot();
  return snapshot;
}
```

Replace the bodies of `printStatus`, `printDebugTo`, and `printBluetoothTelemetryTo` with calls to `TelemetryFormatter`.

Keep `printResetCause` if readable reset tokens are still wanted in `STATUS`; if so, print them before `TelemetryFormatter::printStatus` or add reset-token fields to `TelemetrySnapshot` in this task.

- [ ] **Step 6: Verify and commit telemetry centralization**

Run:

```bash
make -C tests/native all
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 SelfBalanceRobot
```

Expected:
- All native tests print `PASS`.
- Arduino compile succeeds.
- `STATUS`, USB debug, and Bluetooth telemetry still use the same field names needed by `docs/bring-up.md`.

Commit:

```bash
git add SelfBalanceRobot/TelemetryFormatter.h tests/native/test_telemetry_formatter.cpp tests/native/Makefile SelfBalanceRobot/SelfBalanceRobot.ino
git commit -m "refactor: centralize telemetry formatting"
```

---

### Task 6: Documentation and Final Performance Check

**Files:**
- Modify: `docs/bring-up.md`
- Modify: `docs/superpowers/specs/2026-05-29-bluetooth-test-control-channel-design.md`
- Modify: `README.md`

- [ ] **Step 1: Update bring-up diagnostics**

Modify `docs/bring-up.md` in the command and diagnostics sections. Add:

```markdown
`STATUS` also reports runtime loop counters:

- `workUs`: work time for the most recent balance tick.
- `maxWorkUs`: highest observed balance tick work time since boot or counter reset.
- `missed`: count of balance ticks whose interval exceeded `BalanceLoopMicros`.
- `feedbackFull` / `feedbackLight`: encoder feedback refresh counts.
- `motorWrites` / `motorStops`: hardware motor output calls.
- `telemUs` / `maxTelemUs`: latest and peak telemetry formatting time.

When tuning performance, collect one `STATUS` snapshot with debug telemetry off and one with telemetry on. Prefer changes that reduce `maxWorkUs` and keep `missed` stable at zero before considering a faster balance loop.
```

- [ ] **Step 2: Fix Bluetooth serial documentation drift**

In `docs/superpowers/specs/2026-05-29-bluetooth-test-control-channel-design.md`, ensure the architecture and plan text consistently says:

```markdown
Bluetooth test control uses MegaPi `Serial3` through `ROBOT_BLUETOOTH_SERIAL`.
`Serial2` remains reserved for the RPi primary control link.
```

Remove references that say the implementation should start `Serial1`.

- [ ] **Step 3: Update README with the performance baseline**

Modify `README.md`:

```markdown
## Performance Diagnostics

The firmware includes runtime counters in `STATUS` for loop work time, missed balance-loop periods, encoder feedback refreshes, motor output writes, and telemetry print time. Use these counters before changing `BalanceLoopMicros`, PID math, encoder sign conventions, or motor output behavior.
```

- [ ] **Step 4: Run final verification**

Run:

```bash
make -C tests/native all
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 SelfBalanceRobot
git status --short
```

Expected:
- All native tests print `PASS`.
- Arduino compile succeeds.
- Compile output remains comfortably below Mega 2560 flash and SRAM limits.
- `git status --short` shows only files touched by this plan plus any pre-existing unrelated user changes.

- [ ] **Step 5: Commit docs and final verification notes**

Commit:

```bash
git add docs/bring-up.md docs/superpowers/specs/2026-05-29-bluetooth-test-control-channel-design.md README.md
git commit -m "docs: document performance diagnostics"
```

---

## Post-Plan Decision Point

After this plan is implemented, use `STATUS` snapshots to decide whether to continue:

- If `maxWorkUs` is comfortably below `10000` and `missed=0`, keep `BalanceLoopMicros` at `10000UL` and tune balance behavior mechanically/PID-first.
- If `maxWorkUs` is close to `10000` or `missed` rises during telemetry, keep 100 Hz and reduce telemetry further.
- If `maxWorkUs` is consistently low with telemetry off, consider a separate plan for a faster control loop, such as `BalanceLoopMicros = 5000UL`, with hardware-held tests only.

Do not change loop frequency, PID math, encoder direction, or wheel-speed damping signs in this plan.

## Self-Review

- Spec coverage: The plan covers measurement, behavior-preserving loop simplification, balance math extraction, repeated motor-write reduction, optional speed feedback decimation, telemetry cleanup, documentation, and final verification.
- Scope: This is one coherent refactor/performance pass. It does not tune PID, change motor signs, change EEPROM semantics, or restore drive/ultrasonic behavior.
- Type consistency: `RuntimeStatsSnapshot`, `BalancePipelineInput`, `BalancePipelineOutput`, `MotorOutputLatch`, `FeedbackPolicy`, `MotorFeedbackMode`, `FeedbackRequest`, `TelemetrySnapshot`, and `TelemetryFormatter` are defined before later tasks use them.
- Verification: Every implementation task has a focused native test, full native test run, Arduino compile, and commit step.
