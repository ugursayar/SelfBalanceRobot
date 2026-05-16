# MakeBlock Self-Balancing Robot Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the first cautious Arduino sketch for a MakeBlock MegaPi two-wheel self-balancing robot with Bluetooth arming/drive control and ultrasonic obstacle avoidance.

**Architecture:** The project uses an Arduino sketch folder with small C++ modules. Hardware-facing code is isolated from pure control logic so balance math, drive mixing, and state transitions can be tested or inspected independently. The first build implements angle PID balancing and leaves encoder speed feedback as explicit hooks for later.

**Tech Stack:** Arduino C/C++, Makeblock Arduino library (`MeMegaPi.h`), Arduino Mega 2560 board target, optional native C++ checks with `g++`, Git/GitHub.

---

## File Structure

- Create: `.gitignore`
  - Ignores Arduino/build/editor output and the local `.superpowers/brainstorm/` companion files.
- Create: `README.md`
  - Local copy matching the GitHub README, with setup and upload notes.
- Create: `SelfBalanceRobot/SelfBalanceRobot.ino`
  - Arduino entry point. Owns global objects, `setup()`, and `loop()`.
- Create: `SelfBalanceRobot/config.h`
  - All hardware ports, control constants, safety thresholds, feature flags, and motor inversion settings.
- Create: `SelfBalanceRobot/RobotTypes.h`
  - Shared enums and small structs used by all modules.
- Create: `SelfBalanceRobot/BalanceController.h`
- Create: `SelfBalanceRobot/BalanceController.cpp`
  - Fixed-rate angle PID controller with reset and output limiting.
- Create: `SelfBalanceRobot/DriveMixer.h`
- Create: `SelfBalanceRobot/DriveMixer.cpp`
  - Combines balance, forward, and turn commands into left/right motor commands.
- Create: `SelfBalanceRobot/RobotState.h`
- Create: `SelfBalanceRobot/RobotState.cpp`
  - State machine for `DISARMED`, `CALIBRATING`, `BALANCING`, `DRIVE`, and `FAULT`.
- Create: `SelfBalanceRobot/Sensors.h`
- Create: `SelfBalanceRobot/Sensors.cpp`
  - Gyro, ultrasonic, and future encoder hook wrapper using Makeblock APIs.
- Create: `SelfBalanceRobot/BluetoothControl.h`
- Create: `SelfBalanceRobot/BluetoothControl.cpp`
  - Conservative serial command parser for arming, stopping, drive commands, and optional tuning.
- Create: `SelfBalanceRobot/Motors.h`
- Create: `SelfBalanceRobot/Motors.cpp`
  - Motor wrapper around `MeMegaPiDCMotor`, with inversion and safe stop.
- Create: `tests/native/test_balance_controller.cpp`
  - Native checks for PID output sign, limit, reset, and integral behavior.
- Create: `tests/native/test_drive_mixer.cpp`
  - Native checks for balance/forward/turn mixing and saturation.
- Create: `tests/native/test_robot_state.cpp`
  - Native checks for arming, calibration completion, stop, fall fault, and obstacle clamping intent.
- Create: `tests/native/Makefile`
  - Optional local native test build if `g++` is installed.

## Known Hardware API Assumptions

- Include Makeblock library with `#include <MeMegaPi.h>`.
- MegaPi is selected in Arduino IDE as Arduino Mega 2560 or Mega ADK.
- Makeblock motor class: `MeMegaPiDCMotor motor(PORT1B);`, `motor.run(speed);`, `motor.stop();`.
- Makeblock gyro class supports `begin()`, `update()`, and angle accessors such as `angleX()`, `angleY()`, `angleZ()`.
- Makeblock ultrasonic class exposes distance in centimeters through its standard distance method.
- Bluetooth adapter is treated as a serial stream first; if the Makeblock controller packet format requires a dedicated parser later, only `BluetoothControl` changes.

Use these assumptions in code, but keep wrappers narrow so any API correction is local.

---

### Task 1: Local Repository And Project Skeleton

**Files:**
- Create: `.gitignore`
- Create: `README.md`
- Create: `SelfBalanceRobot/SelfBalanceRobot.ino`
- Create: `SelfBalanceRobot/config.h`
- Create: `SelfBalanceRobot/RobotTypes.h`

- [ ] **Step 1: Initialize local git and connect remote**

Run:

```powershell
git init
git branch -M main
git remote add origin https://github.com/ugursayar/SelfBalanceRobot.git
```

Expected: `git status --short` works and the remote is set to `origin`.

- [ ] **Step 2: Create `.gitignore`**

Write this exact file:

```gitignore
.superpowers/brainstorm/
.vscode/
.idea/
build/
*.elf
*.hex
*.bin
*.eep
*.map
*.o
*.d
*.tmp
```

- [ ] **Step 3: Create `README.md`**

Write this exact file:

```markdown
# SelfBalanceRobot

Arduino project for a MakeBlock MegaPi two-wheel self-balancing robot with Bluetooth control and ultrasonic obstacle avoidance.

## Hardware Defaults

- MegaPi programmed as Arduino Mega 2560
- MegaPi Shield for RJ25
- Gyro sensor on RJ25 `PORT_6`
- Ultrasonic sensor on RJ25 `PORT_7`
- Right motor driver on MegaPi port 1
- Left motor driver on MegaPi port 2
- Standard MakeBlock Bluetooth adapter/controller

## Arduino Setup

1. Install the Arduino IDE.
2. Install the Makeblock Arduino library so `MeMegaPi.h` is available.
3. Open `SelfBalanceRobot/SelfBalanceRobot.ino`.
4. Select `Arduino Mega 2560 or Mega ADK`.
5. Select the MegaPi serial port.
6. Upload.

Start with the robot held securely. The sketch boots with motors disabled and requires an arm command before calibration and balancing.
```

- [ ] **Step 4: Create `SelfBalanceRobot/config.h`**

Write this exact file:

```cpp
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <MeMegaPi.h>

namespace Config {
  constexpr uint8_t GyroPort = PORT_6;
  constexpr uint8_t UltrasonicPort = PORT_7;

  constexpr uint8_t RightMotorPort = PORT1B;
  constexpr uint8_t LeftMotorPort = PORT2B;

  constexpr bool InvertRightMotor = false;
  constexpr bool InvertLeftMotor = true;

  constexpr unsigned long BalanceLoopMicros = 10000UL;
  constexpr unsigned long UltrasonicPeriodMillis = 80UL;
  constexpr unsigned long CommandTimeoutMillis = 600UL;
  constexpr unsigned long CalibrationMillis = 1200UL;

  constexpr float BalanceKp = 18.0f;
  constexpr float BalanceKi = 0.0f;
  constexpr float BalanceKd = 0.8f;

  constexpr float MaxTargetLeanDegrees = 5.0f;
  constexpr float FallAngleDegrees = 35.0f;
  constexpr float StillAngleDeltaDegrees = 4.0f;
  constexpr float ObstacleStopDistanceCm = 25.0f;

  constexpr int16_t MotorDeadband = 8;
  constexpr int16_t MaxMotorCommand = 160;
  constexpr int16_t MaxTurnCommand = 50;
  constexpr int16_t MaxDriveCommand = 50;

  constexpr bool EnableRuntimeTuning = true;
  constexpr bool EnableDebugSerial = true;
  constexpr unsigned long DebugPeriodMillis = 200UL;
}

#endif
```

- [ ] **Step 5: Create `SelfBalanceRobot/RobotTypes.h`**

Write this exact file:

```cpp
#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <Arduino.h>

enum class RobotMode : uint8_t {
  Disarmed,
  Calibrating,
  Balancing,
  Drive,
  Fault
};

struct SensorFrame {
  float angleDegrees = 0.0f;
  float distanceCm = 400.0f;
  bool gyroFresh = false;
  bool ultrasonicFresh = false;
  unsigned long nowMillis = 0;
};

struct ControlCommand {
  bool arm = false;
  bool stop = false;
  bool driveEnabled = false;
  int16_t forward = 0;
  int16_t turn = 0;
  float tuneKp = 0.0f;
  float tuneKi = 0.0f;
  float tuneKd = 0.0f;
  bool hasTuning = false;
  unsigned long receivedMillis = 0;
};

struct MotorCommand {
  int16_t left = 0;
  int16_t right = 0;
};

#endif
```

- [ ] **Step 6: Create minimal `SelfBalanceRobot/SelfBalanceRobot.ino`**

Write this exact file:

```cpp
#include "config.h"
#include "RobotTypes.h"

void setup() {
  Serial.begin(115200);
  if (Config::EnableDebugSerial) {
    Serial.println(F("SelfBalanceRobot skeleton ready"));
  }
}

void loop() {
}
```

- [ ] **Step 7: Verify Arduino sketch folder opens**

Run:

```powershell
Get-ChildItem SelfBalanceRobot
```

Expected: the folder contains `SelfBalanceRobot.ino`, `config.h`, and `RobotTypes.h`.

- [ ] **Step 8: Commit skeleton**

Run:

```powershell
git add .gitignore README.md SelfBalanceRobot
git commit -m "chore: scaffold Arduino project"
```

Expected: one local commit is created.

---

### Task 2: Balance Controller

**Files:**
- Create: `SelfBalanceRobot/BalanceController.h`
- Create: `SelfBalanceRobot/BalanceController.cpp`
- Create: `tests/native/test_balance_controller.cpp`
- Create: `tests/native/Makefile`

- [ ] **Step 1: Create native balance controller test**

Write `tests/native/test_balance_controller.cpp`:

```cpp
#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>

#define private public
#include "../../SelfBalanceRobot/BalanceController.h"
#undef private

static void test_output_sign_and_limit() {
  BalanceController controller;
  controller.setTunings(20.0f, 0.0f, 0.0f);
  controller.setOutputLimit(100);
  controller.setTargetAngle(0.0f);

  const int16_t output = controller.update(8.0f, 0.01f);
  assert(output == -100);
}

static void test_reset_clears_integral_and_previous_error() {
  BalanceController controller;
  controller.setTunings(1.0f, 5.0f, 0.5f);
  controller.setOutputLimit(255);
  controller.setTargetAngle(0.0f);
  (void)controller.update(-10.0f, 0.1f);
  controller.reset();
  assert(std::fabs(controller.integral_) < 0.0001f);
  assert(std::fabs(controller.previousError_) < 0.0001f);
  assert(!controller.hasPreviousError_);
}

int main() {
  test_output_sign_and_limit();
  test_reset_clears_integral_and_previous_error();
  std::cout << "test_balance_controller passed\n";
  return 0;
}
```

- [ ] **Step 2: Create native `Makefile`**

Write `tests/native/Makefile`:

```make
CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra -Werror -I../../SelfBalanceRobot

all: test_balance_controller test_drive_mixer test_robot_state

test_balance_controller: test_balance_controller.cpp ../../SelfBalanceRobot/BalanceController.cpp ../../SelfBalanceRobot/BalanceController.h
	$(CXX) $(CXXFLAGS) test_balance_controller.cpp ../../SelfBalanceRobot/BalanceController.cpp -o test_balance_controller
	./test_balance_controller

test_drive_mixer: test_drive_mixer.cpp ../../SelfBalanceRobot/DriveMixer.cpp ../../SelfBalanceRobot/DriveMixer.h
	$(CXX) $(CXXFLAGS) test_drive_mixer.cpp ../../SelfBalanceRobot/DriveMixer.cpp -o test_drive_mixer
	./test_drive_mixer

test_robot_state: test_robot_state.cpp ../../SelfBalanceRobot/RobotState.cpp ../../SelfBalanceRobot/RobotState.h
	$(CXX) $(CXXFLAGS) test_robot_state.cpp ../../SelfBalanceRobot/RobotState.cpp -o test_robot_state
	./test_robot_state

clean:
	rm -f test_balance_controller test_drive_mixer test_robot_state
```

- [ ] **Step 3: Run test to verify it fails**

Run:

```powershell
cd tests\native
make test_balance_controller
cd ..\..
```

Expected: FAIL because `BalanceController.h` does not exist.

- [ ] **Step 4: Create `SelfBalanceRobot/BalanceController.h`**

Write this exact file:

```cpp
#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <stdint.h>

class BalanceController {
public:
  BalanceController();

  void setTunings(float kp, float ki, float kd);
  void setTargetAngle(float targetAngleDegrees);
  void setOutputLimit(int16_t limit);
  void reset();
  int16_t update(float measuredAngleDegrees, float dtSeconds);

private:
  float kp_;
  float ki_;
  float kd_;
  float targetAngleDegrees_;
  float integral_;
  float previousError_;
  int16_t outputLimit_;
  bool hasPreviousError_;
};

#endif
```

- [ ] **Step 5: Create `SelfBalanceRobot/BalanceController.cpp`**

Write this exact file:

```cpp
#include "BalanceController.h"

#include <math.h>

namespace {
float clampFloat(float value, float low, float high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}
}

BalanceController::BalanceController()
  : kp_(0.0f),
    ki_(0.0f),
    kd_(0.0f),
    targetAngleDegrees_(0.0f),
    integral_(0.0f),
    previousError_(0.0f),
    outputLimit_(255),
    hasPreviousError_(false) {
}

void BalanceController::setTunings(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void BalanceController::setTargetAngle(float targetAngleDegrees) {
  targetAngleDegrees_ = targetAngleDegrees;
}

void BalanceController::setOutputLimit(int16_t limit) {
  outputLimit_ = limit < 0 ? -limit : limit;
}

void BalanceController::reset() {
  integral_ = 0.0f;
  previousError_ = 0.0f;
  hasPreviousError_ = false;
}

int16_t BalanceController::update(float measuredAngleDegrees, float dtSeconds) {
  if (dtSeconds <= 0.0f) {
    return 0;
  }

  const float error = targetAngleDegrees_ - measuredAngleDegrees;
  integral_ += error * dtSeconds;
  integral_ = clampFloat(integral_, -50.0f, 50.0f);

  float derivative = 0.0f;
  if (hasPreviousError_) {
    derivative = (error - previousError_) / dtSeconds;
  }

  previousError_ = error;
  hasPreviousError_ = true;

  const float rawOutput = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);
  const float limited = clampFloat(rawOutput, -outputLimit_, outputLimit_);
  return static_cast<int16_t>(limited);
}
```

- [ ] **Step 6: Run balance controller test**

Run:

```powershell
cd tests\native
make test_balance_controller
cd ..\..
```

Expected: PASS and output contains `test_balance_controller passed`.

- [ ] **Step 7: Commit balance controller**

Run:

```powershell
git add SelfBalanceRobot/BalanceController.* tests/native
git commit -m "feat: add balance controller"
```

Expected: one local commit is created.

---

### Task 3: Drive Mixer

**Files:**
- Create: `SelfBalanceRobot/DriveMixer.h`
- Create: `SelfBalanceRobot/DriveMixer.cpp`
- Create: `tests/native/test_drive_mixer.cpp`

- [ ] **Step 1: Create native drive mixer test**

Write `tests/native/test_drive_mixer.cpp`:

```cpp
#include <cassert>
#include <iostream>

#include "../../SelfBalanceRobot/DriveMixer.h"

static void test_balance_goes_to_both_motors() {
  DriveMixer mixer;
  mixer.setLimits(160, 50, 50, 8);
  MotorCommand command = mixer.mix(40, 0, 0);
  assert(command.left == 40);
  assert(command.right == 40);
}

static void test_turn_adds_opposite_offsets() {
  DriveMixer mixer;
  mixer.setLimits(160, 50, 50, 8);
  MotorCommand command = mixer.mix(40, 0, 15);
  assert(command.left == 25);
  assert(command.right == 55);
}

static void test_deadband_and_saturation() {
  DriveMixer mixer;
  mixer.setLimits(100, 50, 50, 8);
  MotorCommand small = mixer.mix(4, 0, 0);
  assert(small.left == 0);
  assert(small.right == 0);

  MotorCommand saturated = mixer.mix(120, 50, 50);
  assert(saturated.left == 100);
  assert(saturated.right == 100);
}

int main() {
  test_balance_goes_to_both_motors();
  test_turn_adds_opposite_offsets();
  test_deadband_and_saturation();
  std::cout << "test_drive_mixer passed\n";
  return 0;
}
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```powershell
cd tests\native
make test_drive_mixer
cd ..\..
```

Expected: FAIL because `DriveMixer.h` does not exist.

- [ ] **Step 3: Create `SelfBalanceRobot/DriveMixer.h`**

Write this exact file:

```cpp
#ifndef DRIVE_MIXER_H
#define DRIVE_MIXER_H

#include "RobotTypes.h"

class DriveMixer {
public:
  DriveMixer();

  void setLimits(int16_t maxMotorCommand, int16_t maxDriveCommand, int16_t maxTurnCommand, int16_t deadband);
  MotorCommand mix(int16_t balanceOutput, int16_t forwardCommand, int16_t turnCommand) const;

private:
  int16_t maxMotorCommand_;
  int16_t maxDriveCommand_;
  int16_t maxTurnCommand_;
  int16_t deadband_;

  int16_t clampValue(int16_t value, int16_t limit) const;
  int16_t applyDeadband(int16_t value) const;
};

#endif
```

- [ ] **Step 4: Create `SelfBalanceRobot/DriveMixer.cpp`**

Write this exact file:

```cpp
#include "DriveMixer.h"

DriveMixer::DriveMixer()
  : maxMotorCommand_(255),
    maxDriveCommand_(80),
    maxTurnCommand_(80),
    deadband_(0) {
}

void DriveMixer::setLimits(int16_t maxMotorCommand, int16_t maxDriveCommand, int16_t maxTurnCommand, int16_t deadband) {
  maxMotorCommand_ = maxMotorCommand;
  maxDriveCommand_ = maxDriveCommand;
  maxTurnCommand_ = maxTurnCommand;
  deadband_ = deadband;
}

MotorCommand DriveMixer::mix(int16_t balanceOutput, int16_t forwardCommand, int16_t turnCommand) const {
  const int16_t drive = clampValue(forwardCommand, maxDriveCommand_);
  const int16_t turn = clampValue(turnCommand, maxTurnCommand_);

  MotorCommand command;
  command.left = applyDeadband(clampValue(balanceOutput + drive - turn, maxMotorCommand_));
  command.right = applyDeadband(clampValue(balanceOutput + drive + turn, maxMotorCommand_));
  return command;
}

int16_t DriveMixer::clampValue(int16_t value, int16_t limit) const {
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

int16_t DriveMixer::applyDeadband(int16_t value) const {
  if (value > -deadband_ && value < deadband_) {
    return 0;
  }
  return value;
}
```

- [ ] **Step 5: Run drive mixer test**

Run:

```powershell
cd tests\native
make test_drive_mixer
cd ..\..
```

Expected: PASS and output contains `test_drive_mixer passed`.

- [ ] **Step 6: Commit drive mixer**

Run:

```powershell
git add SelfBalanceRobot/DriveMixer.* tests/native/test_drive_mixer.cpp tests/native/Makefile
git commit -m "feat: add drive mixer"
```

Expected: one local commit is created.

---

### Task 4: Robot State Machine

**Files:**
- Create: `SelfBalanceRobot/RobotState.h`
- Create: `SelfBalanceRobot/RobotState.cpp`
- Create: `tests/native/test_robot_state.cpp`

- [ ] **Step 1: Create native state machine test**

Write `tests/native/test_robot_state.cpp`:

```cpp
#include <cassert>
#include <iostream>

#include "../../SelfBalanceRobot/RobotState.h"

static SensorFrame frame(float angle, float distance, unsigned long now) {
  SensorFrame sensor;
  sensor.angleDegrees = angle;
  sensor.distanceCm = distance;
  sensor.gyroFresh = true;
  sensor.ultrasonicFresh = true;
  sensor.nowMillis = now;
  return sensor;
}

static void test_arm_enters_calibrating_then_balancing() {
  RobotState state;
  state.configure(35.0f, 25.0f, 1000UL, 600UL);

  ControlCommand command;
  command.arm = true;
  command.receivedMillis = 100UL;
  state.update(frame(0.0f, 100.0f, 100UL), command);
  assert(state.mode() == RobotMode::Calibrating);

  command.arm = false;
  state.update(frame(0.5f, 100.0f, 1200UL), command);
  assert(state.mode() == RobotMode::Balancing);
}

static void test_stop_returns_to_disarmed() {
  RobotState state;
  state.configure(35.0f, 25.0f, 1000UL, 600UL);
  ControlCommand command;
  command.arm = true;
  command.receivedMillis = 0UL;
  state.update(frame(0.0f, 100.0f, 0UL), command);
  state.update(frame(0.0f, 100.0f, 1100UL), command);

  command.stop = true;
  state.update(frame(0.0f, 100.0f, 1200UL), command);
  assert(state.mode() == RobotMode::Disarmed);
  assert(!state.motorsEnabled());
}

static void test_fall_enters_fault() {
  RobotState state;
  state.configure(35.0f, 25.0f, 1000UL, 600UL);
  ControlCommand command;
  command.arm = true;
  command.receivedMillis = 0UL;
  state.update(frame(0.0f, 100.0f, 0UL), command);
  state.update(frame(0.0f, 100.0f, 1100UL), command);

  command.arm = false;
  state.update(frame(45.0f, 100.0f, 1200UL), command);
  assert(state.mode() == RobotMode::Fault);
  assert(!state.motorsEnabled());
}

static void test_obstacle_blocks_forward_only() {
  RobotState state;
  state.configure(35.0f, 25.0f, 1000UL, 600UL);
  ControlCommand command;
  command.arm = true;
  command.driveEnabled = true;
  command.forward = 40;
  command.turn = 10;
  command.receivedMillis = 0UL;
  state.update(frame(0.0f, 100.0f, 0UL), command);
  state.update(frame(0.0f, 10.0f, 1100UL), command);

  ControlCommand safe = state.safeCommand(command, frame(0.0f, 10.0f, 1100UL));
  assert(safe.forward == 0);
  assert(safe.turn == 10);

  command.forward = -40;
  safe = state.safeCommand(command, frame(0.0f, 10.0f, 1200UL));
  assert(safe.forward == -40);
}

int main() {
  test_arm_enters_calibrating_then_balancing();
  test_stop_returns_to_disarmed();
  test_fall_enters_fault();
  test_obstacle_blocks_forward_only();
  std::cout << "test_robot_state passed\n";
  return 0;
}
```

- [ ] **Step 2: Run test to verify it fails**

Run:

```powershell
cd tests\native
make test_robot_state
cd ..\..
```

Expected: FAIL because `RobotState.h` does not exist.

- [ ] **Step 3: Create `SelfBalanceRobot/RobotState.h`**

Write this exact file:

```cpp
#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include "RobotTypes.h"

class RobotState {
public:
  RobotState();

  void configure(float fallAngleDegrees, float obstacleDistanceCm, unsigned long calibrationMillis, unsigned long commandTimeoutMillis);
  void update(const SensorFrame& sensors, const ControlCommand& command);
  ControlCommand safeCommand(const ControlCommand& command, const SensorFrame& sensors) const;

  RobotMode mode() const;
  bool motorsEnabled() const;
  float uprightAngleDegrees() const;

private:
  RobotMode mode_;
  float fallAngleDegrees_;
  float obstacleDistanceCm_;
  unsigned long calibrationMillis_;
  unsigned long commandTimeoutMillis_;
  unsigned long calibrationStartedMillis_;
  float calibrationSum_;
  uint16_t calibrationSamples_;
  float uprightAngleDegrees_;

  bool isFallen(float angleDegrees) const;
  bool commandIsFresh(const ControlCommand& command, unsigned long nowMillis) const;
};

#endif
```

- [ ] **Step 4: Create `SelfBalanceRobot/RobotState.cpp`**

Write this exact file:

```cpp
#include "RobotState.h"

#include <math.h>

RobotState::RobotState()
  : mode_(RobotMode::Disarmed),
    fallAngleDegrees_(35.0f),
    obstacleDistanceCm_(25.0f),
    calibrationMillis_(1000UL),
    commandTimeoutMillis_(600UL),
    calibrationStartedMillis_(0UL),
    calibrationSum_(0.0f),
    calibrationSamples_(0),
    uprightAngleDegrees_(0.0f) {
}

void RobotState::configure(float fallAngleDegrees, float obstacleDistanceCm, unsigned long calibrationMillis, unsigned long commandTimeoutMillis) {
  fallAngleDegrees_ = fallAngleDegrees;
  obstacleDistanceCm_ = obstacleDistanceCm;
  calibrationMillis_ = calibrationMillis;
  commandTimeoutMillis_ = commandTimeoutMillis;
}

void RobotState::update(const SensorFrame& sensors, const ControlCommand& command) {
  if (command.stop) {
    mode_ = RobotMode::Disarmed;
    return;
  }

  if (!sensors.gyroFresh && (mode_ == RobotMode::Balancing || mode_ == RobotMode::Drive)) {
    mode_ = RobotMode::Fault;
    return;
  }

  if ((mode_ == RobotMode::Balancing || mode_ == RobotMode::Drive) && isFallen(sensors.angleDegrees - uprightAngleDegrees_)) {
    mode_ = RobotMode::Fault;
    return;
  }

  switch (mode_) {
    case RobotMode::Disarmed:
      if (command.arm && sensors.gyroFresh) {
        mode_ = RobotMode::Calibrating;
        calibrationStartedMillis_ = sensors.nowMillis;
        calibrationSum_ = 0.0f;
        calibrationSamples_ = 0;
      }
      break;

    case RobotMode::Calibrating:
      if (sensors.gyroFresh) {
        calibrationSum_ += sensors.angleDegrees;
        ++calibrationSamples_;
      }
      if ((sensors.nowMillis - calibrationStartedMillis_) >= calibrationMillis_) {
        if (calibrationSamples_ > 0) {
          uprightAngleDegrees_ = calibrationSum_ / calibrationSamples_;
          mode_ = RobotMode::Balancing;
        } else {
          mode_ = RobotMode::Fault;
        }
      }
      break;

    case RobotMode::Balancing:
      if (command.driveEnabled && commandIsFresh(command, sensors.nowMillis)) {
        mode_ = RobotMode::Drive;
      }
      break;

    case RobotMode::Drive:
      if (!command.driveEnabled || !commandIsFresh(command, sensors.nowMillis)) {
        mode_ = RobotMode::Balancing;
      }
      break;

    case RobotMode::Fault:
      break;
  }
}

ControlCommand RobotState::safeCommand(const ControlCommand& command, const SensorFrame& sensors) const {
  ControlCommand safe = command;
  if (sensors.ultrasonicFresh && sensors.distanceCm > 0.0f && sensors.distanceCm < obstacleDistanceCm_ && safe.forward > 0) {
    safe.forward = 0;
  }
  if (mode_ != RobotMode::Drive) {
    safe.forward = 0;
    safe.turn = 0;
  }
  return safe;
}

RobotMode RobotState::mode() const {
  return mode_;
}

bool RobotState::motorsEnabled() const {
  return mode_ == RobotMode::Balancing || mode_ == RobotMode::Drive;
}

float RobotState::uprightAngleDegrees() const {
  return uprightAngleDegrees_;
}

bool RobotState::isFallen(float angleDegrees) const {
  return fabs(angleDegrees) >= fallAngleDegrees_;
}

bool RobotState::commandIsFresh(const ControlCommand& command, unsigned long nowMillis) const {
  return (nowMillis - command.receivedMillis) <= commandTimeoutMillis_;
}
```

- [ ] **Step 5: Run state machine test**

Run:

```powershell
cd tests\native
make test_robot_state
cd ..\..
```

Expected: PASS and output contains `test_robot_state passed`.

- [ ] **Step 6: Commit state machine**

Run:

```powershell
git add SelfBalanceRobot/RobotState.* tests/native/test_robot_state.cpp tests/native/Makefile
git commit -m "feat: add robot state machine"
```

Expected: one local commit is created.

---

### Task 5: Hardware Wrappers

**Files:**
- Create: `SelfBalanceRobot/Sensors.h`
- Create: `SelfBalanceRobot/Sensors.cpp`
- Create: `SelfBalanceRobot/Motors.h`
- Create: `SelfBalanceRobot/Motors.cpp`
- Create: `SelfBalanceRobot/BluetoothControl.h`
- Create: `SelfBalanceRobot/BluetoothControl.cpp`

- [ ] **Step 1: Create `SelfBalanceRobot/Sensors.h`**

Write this exact file:

```cpp
#ifndef SENSORS_H
#define SENSORS_H

#include "RobotTypes.h"
#include "config.h"

class Sensors {
public:
  Sensors();

  void begin();
  SensorFrame update(unsigned long nowMillis);

private:
  MeGyro gyro_;
  MeUltrasonicSensor ultrasonic_;
  SensorFrame frame_;
  unsigned long lastUltrasonicMillis_;
};

#endif
```

- [ ] **Step 2: Create `SelfBalanceRobot/Sensors.cpp`**

Write this exact file:

```cpp
#include "Sensors.h"

Sensors::Sensors()
  : gyro_(Config::GyroPort),
    ultrasonic_(Config::UltrasonicPort),
    lastUltrasonicMillis_(0UL) {
}

void Sensors::begin() {
  gyro_.begin();
  frame_ = SensorFrame();
}

SensorFrame Sensors::update(unsigned long nowMillis) {
  frame_.nowMillis = nowMillis;

  gyro_.update();
  frame_.angleDegrees = gyro_.angleY();
  frame_.gyroFresh = true;

  if ((nowMillis - lastUltrasonicMillis_) >= Config::UltrasonicPeriodMillis) {
    frame_.distanceCm = ultrasonic_.distanceCm();
    frame_.ultrasonicFresh = true;
    lastUltrasonicMillis_ = nowMillis;
  }

  return frame_;
}
```

- [ ] **Step 3: Create `SelfBalanceRobot/Motors.h`**

Write this exact file:

```cpp
#ifndef MOTORS_H
#define MOTORS_H

#include "RobotTypes.h"
#include "config.h"

class Motors {
public:
  Motors();

  void begin();
  void write(const MotorCommand& command);
  void stop();

private:
  MeMegaPiDCMotor rightMotor_;
  MeMegaPiDCMotor leftMotor_;

  int16_t applyRightDirection(int16_t speed) const;
  int16_t applyLeftDirection(int16_t speed) const;
};

#endif
```

- [ ] **Step 4: Create `SelfBalanceRobot/Motors.cpp`**

Write this exact file:

```cpp
#include "Motors.h"

Motors::Motors()
  : rightMotor_(Config::RightMotorPort),
    leftMotor_(Config::LeftMotorPort) {
}

void Motors::begin() {
  stop();
}

void Motors::write(const MotorCommand& command) {
  rightMotor_.run(applyRightDirection(command.right));
  leftMotor_.run(applyLeftDirection(command.left));
}

void Motors::stop() {
  rightMotor_.stop();
  leftMotor_.stop();
}

int16_t Motors::applyRightDirection(int16_t speed) const {
  return Config::InvertRightMotor ? -speed : speed;
}

int16_t Motors::applyLeftDirection(int16_t speed) const {
  return Config::InvertLeftMotor ? -speed : speed;
}
```

- [ ] **Step 5: Create `SelfBalanceRobot/BluetoothControl.h`**

Write this exact file:

```cpp
#ifndef BLUETOOTH_CONTROL_H
#define BLUETOOTH_CONTROL_H

#include "RobotTypes.h"
#include "config.h"

class BluetoothControl {
public:
  BluetoothControl();

  void begin(Stream& stream);
  ControlCommand update(unsigned long nowMillis);
  const ControlCommand& current() const;

private:
  Stream* stream_;
  ControlCommand command_;
  char buffer_[48];
  uint8_t index_;

  void parseLine(char* line, unsigned long nowMillis);
  int16_t parseCommandValue(const char* text, int16_t low, int16_t high) const;
};

#endif
```

- [ ] **Step 6: Create `SelfBalanceRobot/BluetoothControl.cpp`**

Write this exact file:

```cpp
#include "BluetoothControl.h"

#include <stdlib.h>
#include <string.h>

BluetoothControl::BluetoothControl()
  : stream_(nullptr),
    index_(0) {
}

void BluetoothControl::begin(Stream& stream) {
  stream_ = &stream;
  command_ = ControlCommand();
  index_ = 0;
}

ControlCommand BluetoothControl::update(unsigned long nowMillis) {
  if (stream_ == nullptr) {
    return command_;
  }

  while (stream_->available() > 0) {
    const char c = static_cast<char>(stream_->read());
    if (c == '\n' || c == '\r') {
      if (index_ > 0) {
        buffer_[index_] = '\0';
        parseLine(buffer_, nowMillis);
        index_ = 0;
      }
    } else if (index_ < sizeof(buffer_) - 1) {
      buffer_[index_++] = c;
    }
  }

  return command_;
}

const ControlCommand& BluetoothControl::current() const {
  return command_;
}

void BluetoothControl::parseLine(char* line, unsigned long nowMillis) {
  command_.arm = false;
  command_.stop = false;

  if (strcmp(line, "ARM") == 0) {
    command_.arm = true;
    command_.receivedMillis = nowMillis;
    return;
  }

  if (strcmp(line, "STOP") == 0) {
    command_.stop = true;
    command_.driveEnabled = false;
    command_.forward = 0;
    command_.turn = 0;
    command_.receivedMillis = nowMillis;
    return;
  }

  if (strcmp(line, "BALANCE") == 0) {
    command_.driveEnabled = false;
    command_.forward = 0;
    command_.turn = 0;
    command_.receivedMillis = nowMillis;
    return;
  }

  if (strncmp(line, "DRIVE ", 6) == 0) {
    char* forwardText = strtok(line + 6, " ");
    char* turnText = strtok(nullptr, " ");
    if (forwardText != nullptr && turnText != nullptr) {
      command_.driveEnabled = true;
      command_.forward = parseCommandValue(forwardText, -Config::MaxDriveCommand, Config::MaxDriveCommand);
      command_.turn = parseCommandValue(turnText, -Config::MaxTurnCommand, Config::MaxTurnCommand);
      command_.receivedMillis = nowMillis;
    }
    return;
  }

  if (Config::EnableRuntimeTuning && strncmp(line, "PID ", 4) == 0) {
    char* kpText = strtok(line + 4, " ");
    char* kiText = strtok(nullptr, " ");
    char* kdText = strtok(nullptr, " ");
    if (kpText != nullptr && kiText != nullptr && kdText != nullptr) {
      command_.tuneKp = atof(kpText);
      command_.tuneKi = atof(kiText);
      command_.tuneKd = atof(kdText);
      command_.hasTuning = true;
      command_.receivedMillis = nowMillis;
    }
  }
}

int16_t BluetoothControl::parseCommandValue(const char* text, int16_t low, int16_t high) const {
  long value = atol(text);
  if (value < low) {
    value = low;
  }
  if (value > high) {
    value = high;
  }
  return static_cast<int16_t>(value);
}
```

- [ ] **Step 7: Inspect for Makeblock API mismatches before upload**

Open the installed Makeblock library examples and confirm these names:

```text
MeGyro
MeGyro::begin()
MeGyro::update()
MeGyro::angleY()
MeUltrasonicSensor
MeUltrasonicSensor::distanceCm()
MeMegaPiDCMotor
MeMegaPiDCMotor::run()
MeMegaPiDCMotor::stop()
```

If the local examples use `MeGyro gyro(PORT_6, 0x69)` or `distanceCm(400)`, adjust only `Sensors.h` and `Sensors.cpp`.

- [ ] **Step 8: Commit hardware wrappers**

Run:

```powershell
git add SelfBalanceRobot/Sensors.* SelfBalanceRobot/Motors.* SelfBalanceRobot/BluetoothControl.*
git commit -m "feat: add hardware wrappers"
```

Expected: one local commit is created.

---

### Task 6: Integrate Arduino Sketch

**Files:**
- Modify: `SelfBalanceRobot/SelfBalanceRobot.ino`

- [ ] **Step 1: Replace `SelfBalanceRobot/SelfBalanceRobot.ino` with integrated sketch**

Write this exact file:

```cpp
#include "BalanceController.h"
#include "BluetoothControl.h"
#include "DriveMixer.h"
#include "Motors.h"
#include "RobotState.h"
#include "Sensors.h"
#include "config.h"

Sensors sensors;
Motors motors;
BluetoothControl bluetooth;
BalanceController balance;
DriveMixer mixer;
RobotState robotState;

unsigned long lastBalanceMicros = 0UL;
unsigned long lastDebugMillis = 0UL;

void printDebug(const SensorFrame& frame, const ControlCommand& command, const MotorCommand& motorsOut) {
  if (!Config::EnableDebugSerial) {
    return;
  }

  const unsigned long now = millis();
  if ((now - lastDebugMillis) < Config::DebugPeriodMillis) {
    return;
  }
  lastDebugMillis = now;

  Serial.print(F("mode="));
  Serial.print(static_cast<int>(robotState.mode()));
  Serial.print(F(" angle="));
  Serial.print(frame.angleDegrees);
  Serial.print(F(" upright="));
  Serial.print(robotState.uprightAngleDegrees());
  Serial.print(F(" dist="));
  Serial.print(frame.distanceCm);
  Serial.print(F(" fwd="));
  Serial.print(command.forward);
  Serial.print(F(" turn="));
  Serial.print(command.turn);
  Serial.print(F(" left="));
  Serial.print(motorsOut.left);
  Serial.print(F(" right="));
  Serial.println(motorsOut.right);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  sensors.begin();
  motors.begin();
  bluetooth.begin(Serial1);

  balance.setTunings(Config::BalanceKp, Config::BalanceKi, Config::BalanceKd);
  balance.setOutputLimit(Config::MaxMotorCommand);

  mixer.setLimits(Config::MaxMotorCommand, Config::MaxDriveCommand, Config::MaxTurnCommand, Config::MotorDeadband);
  robotState.configure(
    Config::FallAngleDegrees,
    Config::ObstacleStopDistanceCm,
    Config::CalibrationMillis,
    Config::CommandTimeoutMillis
  );

  lastBalanceMicros = micros();

  if (Config::EnableDebugSerial) {
    Serial.println(F("SelfBalanceRobot ready. Send ARM to calibrate, STOP to disarm, DRIVE fwd turn to drive."));
  }
}

void loop() {
  const unsigned long nowMicros = micros();
  if ((nowMicros - lastBalanceMicros) < Config::BalanceLoopMicros) {
    return;
  }

  const float dtSeconds = (nowMicros - lastBalanceMicros) / 1000000.0f;
  lastBalanceMicros = nowMicros;

  const unsigned long nowMillis = millis();
  SensorFrame frame = sensors.update(nowMillis);
  ControlCommand command = bluetooth.update(nowMillis);

  if (command.hasTuning) {
    balance.setTunings(command.tuneKp, command.tuneKi, command.tuneKd);
  }

  robotState.update(frame, command);

  MotorCommand motorsOut;
  if (robotState.motorsEnabled()) {
    ControlCommand safe = robotState.safeCommand(command, frame);
    const float targetAngle = robotState.uprightAngleDegrees()
      + ((static_cast<float>(safe.forward) / Config::MaxDriveCommand) * Config::MaxTargetLeanDegrees);
    balance.setTargetAngle(targetAngle);
    const int16_t balanceOutput = balance.update(frame.angleDegrees, dtSeconds);
    motorsOut = mixer.mix(balanceOutput, 0, safe.turn);
    motors.write(motorsOut);
  } else {
    balance.reset();
    motors.stop();
  }

  printDebug(frame, command, motorsOut);
}
```

- [ ] **Step 2: Run native tests**

Run:

```powershell
cd tests\native
make
cd ..\..
```

Expected: all existing native tests pass. If `make` or `g++` is unavailable, record that native tests were skipped and continue to Arduino IDE compile.

- [ ] **Step 3: Compile in Arduino IDE**

Open:

```text
SelfBalanceRobot/SelfBalanceRobot.ino
```

Select:

```text
Board: Arduino Mega 2560 or Mega ADK
Port: the MegaPi COM port
```

Click Verify.

Expected: compile succeeds. If it fails on Makeblock API names, fix only the hardware wrapper module named in the error.

- [ ] **Step 4: Commit integrated sketch**

Run:

```powershell
git add SelfBalanceRobot
git commit -m "feat: integrate balancing sketch"
```

Expected: one local commit is created.

---

### Task 7: Bring-Up Notes And GitHub Push

**Files:**
- Create: `docs/bring-up.md`
- Modify: `README.md`

- [ ] **Step 1: Create `docs/bring-up.md`**

Write this exact file:

```markdown
# Bring-Up Guide

Use short tests and keep the robot held securely until motor direction and balance correction direction are confirmed.

## Serial Commands

- `ARM`: calibrate upright angle and enter balancing when calibration completes.
- `STOP`: stop motors and return to disarmed.
- `BALANCE`: leave drive mode but keep balancing.
- `DRIVE <forward> <turn>`: enable drive mode. Example: `DRIVE 20 0`.
- `PID <kp> <ki> <kd>`: update balance gains when runtime tuning is enabled.

## First Checks

1. Upload with wheels off the ground.
2. Open Serial Monitor at 115200 baud.
3. Confirm debug output shows changing angle and distance values.
4. Send `ARM` while holding the robot upright and still.
5. Tilt the robot gently and confirm wheels correct in the direction that would drive under the falling body.
6. If either wheel runs backward, change `InvertRightMotor` or `InvertLeftMotor` in `config.h`.
7. If both wheels correct the wrong way, invert the sign of the selected gyro angle in `Sensors.cpp`.

## Balance Tuning

Start with small tests:

1. Keep `BalanceKi` at `0.0`.
2. Increase `BalanceKp` until the robot strongly corrects but does not oscillate violently.
3. Increase `BalanceKd` to damp oscillation.
4. Add a very small `BalanceKi` only if the robot consistently leans after proportional and derivative tuning.

## Obstacle Check

With the robot balancing, place an obstacle closer than the configured threshold and send `DRIVE 30 0`. Forward command should be clamped. `DRIVE -30 0` should still be allowed.
```

- [ ] **Step 2: Update `README.md`**

Append this section:

```markdown

## Bring-Up

Read `docs/bring-up.md` before enabling the motors on the floor. Start with the robot held securely and use `STOP` immediately if correction direction is wrong.
```

- [ ] **Step 3: Commit docs**

Run:

```powershell
git add README.md docs/bring-up.md docs/superpowers/specs/2026-05-16-makeblock-self-balancing-robot-design.md docs/superpowers/plans/2026-05-16-makeblock-self-balancing-robot.md
git commit -m "docs: add bring-up guide and implementation plan"
```

Expected: one local commit is created.

- [ ] **Step 4: Push to GitHub**

Run:

```powershell
git pull --rebase origin main
git push -u origin main
```

Expected: GitHub contains the Arduino sketch, design spec, implementation plan, and bring-up guide.

---

## Final Verification

- [ ] `git status --short` is clean after push.
- [ ] `SelfBalanceRobot/SelfBalanceRobot.ino` opens in Arduino IDE.
- [ ] Arduino Verify succeeds for Arduino Mega 2560 or Mega ADK.
- [ ] Native tests pass if `g++` and `make` are available.
- [ ] GitHub repo shows latest commit on `main`.

## Self-Review Notes

- Spec coverage: hardware mapping, safe startup, calibration, angle PID, Bluetooth arming/drive, ultrasonic forward clamp, motor inversion config, and bring-up flow are all covered.
- Remaining hardware uncertainty: exact Makeblock gyro and ultrasonic method names must be confirmed against the locally installed Makeblock library during Task 5 or Arduino Verify in Task 6.
- Scope: this plan intentionally excludes encoder speed feedback from the first build and leaves it as a later task, matching the cautious first-build selection.
