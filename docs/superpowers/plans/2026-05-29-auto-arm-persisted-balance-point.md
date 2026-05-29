# Auto-Arm Persisted Balance Point Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add cable-free auto-arm using a persisted absolute gyro balance point, and learn updated balance points conservatively during stable tests.

**Architecture:** Add three focused modules: `AutoArmController` decides when stillness near the stored angle is good enough to arm, `BalancePointStore` validates and persists an EEPROM record through a byte-storage interface, and `BalancePointLearner` decides when a stable balancing session should save a smoothed new point. `SelfBalanceRobot.ino` wires these into the existing state machine and balance loop.

**Tech Stack:** Arduino C++11 for MegaPi/Mega 2560, EEPROM via Arduino `EEPROM.update`, native C++ tests through `tests/native/Makefile`, Arduino CLI compile for `arduino:avr:mega`.

---

## File Structure

- Create `SelfBalanceRobot/AutoArmController.h` and `.cpp`: pure C++ stillness gate for auto-arm.
- Create `SelfBalanceRobot/BalancePointStore.h` and `.cpp`: EEPROM record validation, checksum, and save/load against a byte-storage interface.
- Create `SelfBalanceRobot/EepromByteStorage.h`: Arduino EEPROM adapter used by the sketch.
- Create `SelfBalanceRobot/BalancePointLearner.h` and `.cpp`: pure C++ stability gate for learning.
- Modify `SelfBalanceRobot/RobotState.h` and `.cpp`: add a public direct entry into balancing at an absolute target.
- Modify `SelfBalanceRobot/config.h`: add auto-arm, EEPROM, and learning thresholds.
- Modify `SelfBalanceRobot/SelfBalanceRobot.ino`: load the persisted point, auto-arm from `Disarmed`, use persisted target semantics, suppress on stop, and save learned points.
- Modify `tests/native/Makefile`: build new native tests.
- Create `tests/native/test_auto_arm_controller.cpp`.
- Create `tests/native/test_balance_point_store.cpp`.
- Create `tests/native/test_balance_point_learner.cpp`.
- Modify `tests/native/test_robot_state.cpp`: cover direct balancing entry.
- Modify `docs/bring-up.md`: document cable-free auto-arm and EEPROM behavior.

---

### Task 1: Auto-Arm Stillness Gate

**Files:**
- Create: `SelfBalanceRobot/AutoArmController.h`
- Create: `SelfBalanceRobot/AutoArmController.cpp`
- Create: `tests/native/test_auto_arm_controller.cpp`
- Modify: `tests/native/Makefile`

- [ ] **Step 1: Write the failing auto-arm controller tests**

Create `tests/native/test_auto_arm_controller.cpp`:

```cpp
#include <cassert>
#include <cstdlib>
#include <iostream>

#include "../../SelfBalanceRobot/AutoArmController.h"

static SensorFrame frame(float angle, float rate, unsigned long now,
                         bool fresh = true) {
  SensorFrame f;
  f.angleDegrees = angle;
  f.angleRateDegPerSec = rate;
  f.nowMillis = now;
  f.gyroFresh = fresh;
  return f;
}

static AutoArmController configured() {
  AutoArmController controller;
  controller.configure(2.0f, 5.0f, 500);
  controller.setTargetBalancePoint(0.7f);
  return controller;
}

static void test_does_not_trigger_outside_angle_window() {
  AutoArmController controller = configured();

  assert(!controller.update(frame(3.0f, 0.0f, 0)));
  assert(!controller.update(frame(3.0f, 0.0f, 600)));
}

static void test_does_not_trigger_when_rate_is_too_high() {
  AutoArmController controller = configured();

  assert(!controller.update(frame(0.8f, 8.0f, 0)));
  assert(!controller.update(frame(0.8f, 8.0f, 600)));
}

static void test_triggers_after_stillness_duration() {
  AutoArmController controller = configured();

  assert(!controller.update(frame(0.8f, 1.0f, 1000)));
  assert(!controller.update(frame(0.6f, 0.5f, 1300)));
  assert(controller.update(frame(0.7f, 0.0f, 1500)));
}

static void test_resets_candidate_when_motion_breaks_stillness() {
  AutoArmController controller = configured();

  assert(!controller.update(frame(0.8f, 0.0f, 0)));
  assert(!controller.update(frame(0.9f, 10.0f, 300)));
  assert(!controller.update(frame(0.8f, 0.0f, 600)));
  assert(!controller.update(frame(0.8f, 0.0f, 900)));
  assert(controller.update(frame(0.8f, 0.0f, 1100)));
}

static void test_stop_cooldown_suppresses_auto_arm_until_expired() {
  AutoArmController controller = configured();
  controller.suppressUntil(1000, 1000);

  assert(!controller.update(frame(0.7f, 0.0f, 1200)));
  assert(!controller.update(frame(0.7f, 0.0f, 1900)));
  assert(!controller.update(frame(0.7f, 0.0f, 2000)));
  assert(controller.update(frame(0.7f, 0.0f, 2500)));
}

int main() {
  test_does_not_trigger_outside_angle_window();
  test_does_not_trigger_when_rate_is_too_high();
  test_triggers_after_stillness_duration();
  test_resets_candidate_when_motion_breaks_stillness();
  test_stop_cooldown_suppresses_auto_arm_until_expired();

  std::cout << "test_auto_arm_controller PASS\n";
  return EXIT_SUCCESS;
}
```

- [ ] **Step 2: Add the Makefile target and verify the test fails**

Modify `tests/native/Makefile`:

```make
.PHONY: all test_balance_controller test_robot_state test_sensors test_auto_arm_controller clean

all: test_balance_controller test_robot_state test_sensors test_auto_arm_controller

$(BUILD_DIR)/test_auto_arm_controller: test_auto_arm_controller.cpp $(ROOT)/SelfBalanceRobot/AutoArmController.cpp $(ROOT)/SelfBalanceRobot/AutoArmController.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_auto_arm_controller.cpp $(ROOT)/SelfBalanceRobot/AutoArmController.cpp -o $@

test_auto_arm_controller: $(BUILD_DIR)/test_auto_arm_controller
	./$(BUILD_DIR)/test_auto_arm_controller
```

Run:

```bash
make test_auto_arm_controller
```

Expected: FAIL because `AutoArmController.h` does not exist.

- [ ] **Step 3: Implement `AutoArmController`**

Create `SelfBalanceRobot/AutoArmController.h`:

```cpp
#ifndef AUTO_ARM_CONTROLLER_H
#define AUTO_ARM_CONTROLLER_H

#include "RobotTypes.h"

class AutoArmController {
public:
  AutoArmController();

  void configure(float angleWindowDegrees, float maxRateDegPerSec,
                 unsigned long stillMillis);
  void setTargetBalancePoint(float balancePointDegrees);
  void reset();
  void suppressUntil(unsigned long nowMillis, unsigned long cooldownMillis);
  bool update(const SensorFrame& frame);

private:
  bool frameIsStillCandidate(const SensorFrame& frame) const;

  float targetBalancePointDegrees_;
  float angleWindowDegrees_;
  float maxRateDegPerSec_;
  unsigned long stillMillis_;
  unsigned long candidateStartMillis_;
  unsigned long suppressUntilMillis_;
  bool hasCandidate_;
};

#endif
```

Create `SelfBalanceRobot/AutoArmController.cpp`:

```cpp
#include "AutoArmController.h"

#include <math.h>

AutoArmController::AutoArmController()
    : targetBalancePointDegrees_(0.0f), angleWindowDegrees_(2.0f),
      maxRateDegPerSec_(5.0f), stillMillis_(500),
      candidateStartMillis_(0), suppressUntilMillis_(0),
      hasCandidate_(false) {}

void AutoArmController::configure(float angleWindowDegrees,
                                  float maxRateDegPerSec,
                                  unsigned long stillMillis) {
  angleWindowDegrees_ = angleWindowDegrees < 0.0f
                            ? -angleWindowDegrees
                            : angleWindowDegrees;
  maxRateDegPerSec_ = maxRateDegPerSec < 0.0f
                          ? -maxRateDegPerSec
                          : maxRateDegPerSec;
  stillMillis_ = stillMillis;
  reset();
}

void AutoArmController::setTargetBalancePoint(float balancePointDegrees) {
  targetBalancePointDegrees_ = balancePointDegrees;
  reset();
}

void AutoArmController::reset() {
  hasCandidate_ = false;
  candidateStartMillis_ = 0;
}

void AutoArmController::suppressUntil(unsigned long nowMillis,
                                      unsigned long cooldownMillis) {
  suppressUntilMillis_ = nowMillis + cooldownMillis;
  reset();
}

bool AutoArmController::update(const SensorFrame& frame) {
  if (static_cast<long>(frame.nowMillis - suppressUntilMillis_) < 0) {
    reset();
    return false;
  }

  if (!frameIsStillCandidate(frame)) {
    reset();
    return false;
  }

  if (!hasCandidate_) {
    candidateStartMillis_ = frame.nowMillis;
    hasCandidate_ = true;
  }

  return frame.nowMillis - candidateStartMillis_ >= stillMillis_;
}

bool AutoArmController::frameIsStillCandidate(const SensorFrame& frame) const {
  if (!frame.gyroFresh) {
    return false;
  }

  const float angleError = fabs(frame.angleDegrees - targetBalancePointDegrees_);
  const float rate = fabs(frame.angleRateDegPerSec);
  return angleError <= angleWindowDegrees_ && rate <= maxRateDegPerSec_;
}
```

- [ ] **Step 4: Verify auto-arm controller tests pass**

Run:

```bash
make test_auto_arm_controller
```

Expected: PASS with `test_auto_arm_controller PASS`.

- [ ] **Step 5: Commit**

```bash
git add SelfBalanceRobot/AutoArmController.* tests/native/test_auto_arm_controller.cpp tests/native/Makefile
git commit -m "feat: add auto-arm stillness gate"
```

---

### Task 2: EEPROM Balance Point Store

**Files:**
- Create: `SelfBalanceRobot/BalancePointStore.h`
- Create: `SelfBalanceRobot/BalancePointStore.cpp`
- Create: `SelfBalanceRobot/EepromByteStorage.h`
- Create: `tests/native/test_balance_point_store.cpp`
- Modify: `tests/native/Makefile`

- [ ] **Step 1: Write the failing balance point store tests**

Create `tests/native/test_balance_point_store.cpp`:

```cpp
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "../../SelfBalanceRobot/BalancePointStore.h"

class FakeStorage : public BalancePointStorage {
public:
  FakeStorage() : bytes_(64, 0xFF) {}

  uint8_t read(uint16_t address) const {
    return bytes_[address];
  }

  void update(uint16_t address, uint8_t value) {
    bytes_[address] = value;
  }

  void corrupt(uint16_t address) {
    bytes_[address] ^= 0x55;
  }

private:
  std::vector<uint8_t> bytes_;
};

static BalancePointStore storeFor(FakeStorage& storage) {
  BalancePointStore store(storage, 0);
  store.configure(-12.0f, 12.0f);
  return store;
}

static void test_empty_eeprom_uses_fallback() {
  FakeStorage storage;
  BalancePointStore store = storeFor(storage);

  assert(!store.begin(0.7f));
  assert(!store.hasStoredBalancePoint());
  assert(store.balancePointDegrees() == 0.7f);
}

static void test_save_and_reload_valid_balance_point() {
  FakeStorage storage;
  BalancePointStore writer = storeFor(storage);
  assert(!writer.begin(0.7f));
  assert(writer.saveBalancePoint(1.25f));
  assert(writer.hasStoredBalancePoint());
  assert(writer.balancePointDegrees() == 1.25f);
  assert(writer.writeCounter() == 1);

  BalancePointStore reader = storeFor(storage);
  assert(reader.begin(0.0f));
  assert(reader.hasStoredBalancePoint());
  assert(reader.balancePointDegrees() == 1.25f);
  assert(reader.writeCounter() == 1);
}

static void test_invalid_checksum_is_rejected() {
  FakeStorage storage;
  BalancePointStore writer = storeFor(storage);
  assert(!writer.begin(0.7f));
  assert(writer.saveBalancePoint(-0.5f));

  storage.corrupt(5);

  BalancePointStore reader = storeFor(storage);
  assert(!reader.begin(0.7f));
  assert(!reader.hasStoredBalancePoint());
  assert(reader.balancePointDegrees() == 0.7f);
}

static void test_out_of_range_values_are_not_saved() {
  FakeStorage storage;
  BalancePointStore store = storeFor(storage);
  assert(!store.begin(0.7f));

  assert(!store.saveBalancePoint(20.0f));
  assert(!store.hasStoredBalancePoint());
  assert(store.balancePointDegrees() == 0.7f);
}

int main() {
  test_empty_eeprom_uses_fallback();
  test_save_and_reload_valid_balance_point();
  test_invalid_checksum_is_rejected();
  test_out_of_range_values_are_not_saved();

  std::cout << "test_balance_point_store PASS\n";
  return EXIT_SUCCESS;
}
```

- [ ] **Step 2: Add the Makefile target and verify the test fails**

Modify `tests/native/Makefile`:

```make
.PHONY: all test_balance_controller test_robot_state test_sensors test_auto_arm_controller test_balance_point_store clean

all: test_balance_controller test_robot_state test_sensors test_auto_arm_controller test_balance_point_store

$(BUILD_DIR)/test_balance_point_store: test_balance_point_store.cpp $(ROOT)/SelfBalanceRobot/BalancePointStore.cpp $(ROOT)/SelfBalanceRobot/BalancePointStore.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(ROOT)/SelfBalanceRobot test_balance_point_store.cpp $(ROOT)/SelfBalanceRobot/BalancePointStore.cpp -o $@

test_balance_point_store: $(BUILD_DIR)/test_balance_point_store
	./$(BUILD_DIR)/test_balance_point_store
```

Run:

```bash
make test_balance_point_store
```

Expected: FAIL because `BalancePointStore.h` does not exist.

- [ ] **Step 3: Implement `BalancePointStore` and the EEPROM adapter**

Create `SelfBalanceRobot/BalancePointStore.h`:

```cpp
#ifndef BALANCE_POINT_STORE_H
#define BALANCE_POINT_STORE_H

#include <stdint.h>

class BalancePointStorage {
public:
  virtual ~BalancePointStorage() {}
  virtual uint8_t read(uint16_t address) const = 0;
  virtual void update(uint16_t address, uint8_t value) = 0;
};

class BalancePointStore {
public:
  BalancePointStore(BalancePointStorage& storage, uint16_t baseAddress);

  void configure(float minDegrees, float maxDegrees);
  bool begin(float fallbackBalancePointDegrees);
  bool hasStoredBalancePoint() const;
  float balancePointDegrees() const;
  uint32_t writeCounter() const;
  bool saveBalancePoint(float balancePointDegrees);

private:
  bool readRecord(float& balancePointDegrees, uint32_t& writeCounter) const;
  void writeRecord(float balancePointDegrees, uint32_t writeCounter);
  bool isValidAngle(float degrees) const;
  uint8_t checksum() const;
  uint32_t readUint32(uint16_t offset) const;
  void writeUint32(uint16_t offset, uint32_t value);
  float readFloat(uint16_t offset) const;
  void writeFloat(uint16_t offset, float value);

  BalancePointStorage& storage_;
  uint16_t baseAddress_;
  float minDegrees_;
  float maxDegrees_;
  float balancePointDegrees_;
  uint32_t writeCounter_;
  bool hasStoredBalancePoint_;
};

#endif
```

Create `SelfBalanceRobot/BalancePointStore.cpp`:

```cpp
#include "BalancePointStore.h"

namespace {
const uint32_t kMagic = 0x53425242UL; // "SBRB"
const uint8_t kVersion = 1;
const uint16_t kMagicOffset = 0;
const uint16_t kVersionOffset = 4;
const uint16_t kAngleOffset = 5;
const uint16_t kCounterOffset = 9;
const uint16_t kChecksumOffset = 13;
}

BalancePointStore::BalancePointStore(BalancePointStorage& storage,
                                     uint16_t baseAddress)
    : storage_(storage), baseAddress_(baseAddress), minDegrees_(-12.0f),
      maxDegrees_(12.0f), balancePointDegrees_(0.0f), writeCounter_(0),
      hasStoredBalancePoint_(false) {}

void BalancePointStore::configure(float minDegrees, float maxDegrees) {
  minDegrees_ = minDegrees;
  maxDegrees_ = maxDegrees;
}

bool BalancePointStore::begin(float fallbackBalancePointDegrees) {
  float loadedDegrees = fallbackBalancePointDegrees;
  uint32_t loadedCounter = 0;
  hasStoredBalancePoint_ = readRecord(loadedDegrees, loadedCounter);
  balancePointDegrees_ = hasStoredBalancePoint_ ? loadedDegrees
                                                : fallbackBalancePointDegrees;
  writeCounter_ = hasStoredBalancePoint_ ? loadedCounter : 0;
  return hasStoredBalancePoint_;
}

bool BalancePointStore::hasStoredBalancePoint() const {
  return hasStoredBalancePoint_;
}

float BalancePointStore::balancePointDegrees() const {
  return balancePointDegrees_;
}

uint32_t BalancePointStore::writeCounter() const { return writeCounter_; }

bool BalancePointStore::saveBalancePoint(float balancePointDegrees) {
  if (!isValidAngle(balancePointDegrees)) {
    return false;
  }

  const uint32_t nextCounter = writeCounter_ + 1;
  writeRecord(balancePointDegrees, nextCounter);
  balancePointDegrees_ = balancePointDegrees;
  writeCounter_ = nextCounter;
  hasStoredBalancePoint_ = true;
  return true;
}

bool BalancePointStore::readRecord(float& balancePointDegrees,
                                   uint32_t& writeCounter) const {
  if (readUint32(kMagicOffset) != kMagic) {
    return false;
  }
  if (storage_.read(baseAddress_ + kVersionOffset) != kVersion) {
    return false;
  }
  if (storage_.read(baseAddress_ + kChecksumOffset) != checksum()) {
    return false;
  }

  const float angle = readFloat(kAngleOffset);
  if (!isValidAngle(angle)) {
    return false;
  }

  balancePointDegrees = angle;
  writeCounter = readUint32(kCounterOffset);
  return true;
}

void BalancePointStore::writeRecord(float balancePointDegrees,
                                    uint32_t writeCounter) {
  writeUint32(kMagicOffset, kMagic);
  storage_.update(baseAddress_ + kVersionOffset, kVersion);
  writeFloat(kAngleOffset, balancePointDegrees);
  writeUint32(kCounterOffset, writeCounter);
  storage_.update(baseAddress_ + kChecksumOffset, checksum());
}

bool BalancePointStore::isValidAngle(float degrees) const {
  return degrees == degrees && degrees >= minDegrees_ && degrees <= maxDegrees_;
}

uint8_t BalancePointStore::checksum() const {
  uint8_t value = 0xA5;
  for (uint16_t offset = 0; offset < kChecksumOffset; ++offset) {
    value = static_cast<uint8_t>((value << 1) | (value >> 7));
    value ^= storage_.read(baseAddress_ + offset);
  }
  return value;
}

uint32_t BalancePointStore::readUint32(uint16_t offset) const {
  uint32_t value = 0;
  for (uint8_t i = 0; i < 4; ++i) {
    value |= static_cast<uint32_t>(storage_.read(baseAddress_ + offset + i))
             << (8 * i);
  }
  return value;
}

void BalancePointStore::writeUint32(uint16_t offset, uint32_t value) {
  for (uint8_t i = 0; i < 4; ++i) {
    storage_.update(baseAddress_ + offset + i,
                    static_cast<uint8_t>((value >> (8 * i)) & 0xFF));
  }
}

float BalancePointStore::readFloat(uint16_t offset) const {
  union {
    uint8_t bytes[4];
    float value;
  } data;
  for (uint8_t i = 0; i < 4; ++i) {
    data.bytes[i] = storage_.read(baseAddress_ + offset + i);
  }
  return data.value;
}

void BalancePointStore::writeFloat(uint16_t offset, float value) {
  union {
    uint8_t bytes[4];
    float value;
  } data;
  data.value = value;
  for (uint8_t i = 0; i < 4; ++i) {
    storage_.update(baseAddress_ + offset + i, data.bytes[i]);
  }
}
```

Create `SelfBalanceRobot/EepromByteStorage.h`:

```cpp
#ifndef EEPROM_BYTE_STORAGE_H
#define EEPROM_BYTE_STORAGE_H

#include "BalancePointStore.h"

#include <EEPROM.h>

class EepromByteStorage : public BalancePointStorage {
public:
  uint8_t read(uint16_t address) const {
    return EEPROM.read(address);
  }

  void update(uint16_t address, uint8_t value) {
    EEPROM.update(address, value);
  }
};

#endif
```

- [ ] **Step 4: Verify balance point store tests pass**

Run:

```bash
make test_balance_point_store
```

Expected: PASS with `test_balance_point_store PASS`.

- [ ] **Step 5: Commit**

```bash
git add SelfBalanceRobot/BalancePointStore.* SelfBalanceRobot/EepromByteStorage.h tests/native/test_balance_point_store.cpp tests/native/Makefile
git commit -m "feat: persist balance point records"
```

---

### Task 3: Balance Point Learning Gate

**Files:**
- Create: `SelfBalanceRobot/BalancePointLearner.h`
- Create: `SelfBalanceRobot/BalancePointLearner.cpp`
- Create: `tests/native/test_balance_point_learner.cpp`
- Modify: `tests/native/Makefile`

- [ ] **Step 1: Write the failing learning gate tests**

Create `tests/native/test_balance_point_learner.cpp`:

```cpp
#include <cassert>
#include <cstdlib>
#include <iostream>

#include "../../SelfBalanceRobot/BalancePointLearner.h"

static SensorFrame frame(float angle, float rate, unsigned long now,
                         bool fresh = true) {
  SensorFrame f;
  f.angleDegrees = angle;
  f.angleRateDegPerSec = rate;
  f.nowMillis = now;
  f.gyroFresh = fresh;
  return f;
}

static BalancePointLearner configured() {
  BalancePointLearner learner;
  learner.configure(1000, 500, 2000, 1.0f, 5.0f, 40, 0.25f);
  learner.reset(0.0f, 0);
  return learner;
}

static void test_rejects_before_settle_time() {
  BalancePointLearner learner = configured();

  BalanceLearningResult result =
      learner.update(frame(1.0f, 0.0f, 900), 1.0f, 10, 900);

  assert(!result.shouldSave);
}

static void test_rejects_unstable_angle_rate_and_motor_output() {
  BalancePointLearner learner = configured();

  assert(!learner.update(frame(3.0f, 0.0f, 1200), 1.0f, 10, 1200).shouldSave);
  assert(!learner.update(frame(1.0f, 8.0f, 1800), 1.0f, 10, 1800).shouldSave);
  assert(!learner.update(frame(1.0f, 0.0f, 2400), 1.0f, 80, 2400).shouldSave);
}

static void test_saves_smoothed_point_after_stable_window() {
  BalancePointLearner learner = configured();

  assert(!learner.update(frame(1.0f, 0.0f, 1200), 1.0f, 10, 1200).shouldSave);
  BalanceLearningResult result =
      learner.update(frame(1.0f, 0.0f, 1700), 1.0f, 10, 1700);

  assert(result.shouldSave);
  assert(result.balancePointDegrees == 0.25f);
}

static void test_min_write_interval_prevents_repeated_writes() {
  BalancePointLearner learner = configured();

  assert(!learner.update(frame(1.0f, 0.0f, 1200), 1.0f, 10, 1200).shouldSave);
  assert(learner.update(frame(1.0f, 0.0f, 1700), 1.0f, 10, 1700).shouldSave);
  assert(!learner.update(frame(1.0f, 0.0f, 1900), 1.0f, 10, 1900).shouldSave);
  assert(learner.update(frame(1.0f, 0.0f, 3700), 1.0f, 10, 3700).shouldSave);
}

int main() {
  test_rejects_before_settle_time();
  test_rejects_unstable_angle_rate_and_motor_output();
  test_saves_smoothed_point_after_stable_window();
  test_min_write_interval_prevents_repeated_writes();

  std::cout << "test_balance_point_learner PASS\n";
  return EXIT_SUCCESS;
}
```

- [ ] **Step 2: Add the Makefile target and verify the test fails**

Modify `tests/native/Makefile`:

```make
.PHONY: all test_balance_controller test_robot_state test_sensors test_auto_arm_controller test_balance_point_store test_balance_point_learner clean

all: test_balance_controller test_robot_state test_sensors test_auto_arm_controller test_balance_point_store test_balance_point_learner

$(BUILD_DIR)/test_balance_point_learner: test_balance_point_learner.cpp $(ROOT)/SelfBalanceRobot/BalancePointLearner.cpp $(ROOT)/SelfBalanceRobot/BalancePointLearner.h $(ROOT)/SelfBalanceRobot/RobotTypes.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_balance_point_learner.cpp $(ROOT)/SelfBalanceRobot/BalancePointLearner.cpp -o $@

test_balance_point_learner: $(BUILD_DIR)/test_balance_point_learner
	./$(BUILD_DIR)/test_balance_point_learner
```

Run:

```bash
make test_balance_point_learner
```

Expected: FAIL because `BalancePointLearner.h` does not exist.

- [ ] **Step 3: Implement `BalancePointLearner`**

Create `SelfBalanceRobot/BalancePointLearner.h`:

```cpp
#ifndef BALANCE_POINT_LEARNER_H
#define BALANCE_POINT_LEARNER_H

#include "RobotTypes.h"

struct BalanceLearningResult {
  bool shouldSave = false;
  float balancePointDegrees = 0.0f;
};

class BalancePointLearner {
public:
  BalancePointLearner();

  void configure(unsigned long settleMillis, unsigned long stableMillis,
                 unsigned long minWriteIntervalMillis,
                 float maxAngleErrorDegrees, float maxRateDegPerSec,
                 int16_t maxAbsMotorCommand, float smoothingAlpha);
  void reset(float storedBalancePointDegrees, unsigned long nowMillis);
  BalanceLearningResult update(const SensorFrame& frame,
                               float activeBalancePointDegrees,
                               int16_t balanceOutput,
                               unsigned long nowMillis);

private:
  bool isStable(const SensorFrame& frame, float activeBalancePointDegrees,
                int16_t balanceOutput) const;
  float blend(float activeBalancePointDegrees) const;

  unsigned long settleMillis_;
  unsigned long stableMillis_;
  unsigned long minWriteIntervalMillis_;
  float maxAngleErrorDegrees_;
  float maxRateDegPerSec_;
  int16_t maxAbsMotorCommand_;
  float smoothingAlpha_;
  float storedBalancePointDegrees_;
  unsigned long sessionStartMillis_;
  unsigned long stableStartMillis_;
  unsigned long lastWriteMillis_;
  bool hasStableCandidate_;
  bool hasWritten_;
};

#endif
```

Create `SelfBalanceRobot/BalancePointLearner.cpp`:

```cpp
#include "BalancePointLearner.h"

#include <math.h>

BalancePointLearner::BalancePointLearner()
    : settleMillis_(1000), stableMillis_(500), minWriteIntervalMillis_(30000),
      maxAngleErrorDegrees_(1.0f), maxRateDegPerSec_(5.0f),
      maxAbsMotorCommand_(40), smoothingAlpha_(0.25f),
      storedBalancePointDegrees_(0.0f), sessionStartMillis_(0),
      stableStartMillis_(0), lastWriteMillis_(0),
      hasStableCandidate_(false), hasWritten_(false) {}

void BalancePointLearner::configure(unsigned long settleMillis,
                                    unsigned long stableMillis,
                                    unsigned long minWriteIntervalMillis,
                                    float maxAngleErrorDegrees,
                                    float maxRateDegPerSec,
                                    int16_t maxAbsMotorCommand,
                                    float smoothingAlpha) {
  settleMillis_ = settleMillis;
  stableMillis_ = stableMillis;
  minWriteIntervalMillis_ = minWriteIntervalMillis;
  maxAngleErrorDegrees_ = maxAngleErrorDegrees < 0.0f
                              ? -maxAngleErrorDegrees
                              : maxAngleErrorDegrees;
  maxRateDegPerSec_ = maxRateDegPerSec < 0.0f
                          ? -maxRateDegPerSec
                          : maxRateDegPerSec;
  maxAbsMotorCommand_ = maxAbsMotorCommand < 0
                            ? static_cast<int16_t>(-maxAbsMotorCommand)
                            : maxAbsMotorCommand;
  if (smoothingAlpha < 0.0f) {
    smoothingAlpha_ = 0.0f;
  } else if (smoothingAlpha > 1.0f) {
    smoothingAlpha_ = 1.0f;
  } else {
    smoothingAlpha_ = smoothingAlpha;
  }
}

void BalancePointLearner::reset(float storedBalancePointDegrees,
                                unsigned long nowMillis) {
  storedBalancePointDegrees_ = storedBalancePointDegrees;
  sessionStartMillis_ = nowMillis;
  stableStartMillis_ = 0;
  lastWriteMillis_ = 0;
  hasStableCandidate_ = false;
  hasWritten_ = false;
}

BalanceLearningResult BalancePointLearner::update(
    const SensorFrame& frame, float activeBalancePointDegrees,
    int16_t balanceOutput, unsigned long nowMillis) {
  BalanceLearningResult result;

  if (nowMillis - sessionStartMillis_ < settleMillis_) {
    hasStableCandidate_ = false;
    return result;
  }

  if (!isStable(frame, activeBalancePointDegrees, balanceOutput)) {
    hasStableCandidate_ = false;
    return result;
  }

  if (!hasStableCandidate_) {
    stableStartMillis_ = nowMillis;
    hasStableCandidate_ = true;
  }

  if (nowMillis - stableStartMillis_ < stableMillis_) {
    return result;
  }

  if (hasWritten_ && nowMillis - lastWriteMillis_ < minWriteIntervalMillis_) {
    return result;
  }

  result.shouldSave = true;
  result.balancePointDegrees = blend(activeBalancePointDegrees);
  storedBalancePointDegrees_ = result.balancePointDegrees;
  lastWriteMillis_ = nowMillis;
  hasWritten_ = true;
  return result;
}

bool BalancePointLearner::isStable(const SensorFrame& frame,
                                   float activeBalancePointDegrees,
                                   int16_t balanceOutput) const {
  if (!frame.gyroFresh) {
    return false;
  }

  const int16_t absOutput = balanceOutput < 0
                                ? static_cast<int16_t>(-balanceOutput)
                                : balanceOutput;
  return fabs(frame.angleDegrees - activeBalancePointDegrees) <=
             maxAngleErrorDegrees_ &&
         fabs(frame.angleRateDegPerSec) <= maxRateDegPerSec_ &&
         absOutput <= maxAbsMotorCommand_;
}

float BalancePointLearner::blend(float activeBalancePointDegrees) const {
  return storedBalancePointDegrees_ +
         (smoothingAlpha_ *
          (activeBalancePointDegrees - storedBalancePointDegrees_));
}
```

- [ ] **Step 4: Verify learning tests pass**

Run:

```bash
make test_balance_point_learner
```

Expected: PASS with `test_balance_point_learner PASS`.

- [ ] **Step 5: Commit**

```bash
git add SelfBalanceRobot/BalancePointLearner.* tests/native/test_balance_point_learner.cpp tests/native/Makefile
git commit -m "feat: add balance point learning gate"
```

---

### Task 4: Direct RobotState Entry For Persisted Targets

**Files:**
- Modify: `SelfBalanceRobot/RobotState.h`
- Modify: `SelfBalanceRobot/RobotState.cpp`
- Modify: `tests/native/test_robot_state.cpp`

- [ ] **Step 1: Write the failing RobotState tests**

Add these tests to `tests/native/test_robot_state.cpp` before `main()`:

```cpp
static void test_start_balancing_at_sets_absolute_upright_angle() {
  RobotState state;
  state.configure(30.0f, 4.0f, 100, 150);

  assert(state.startBalancingAt(0.7f));

  assert(state.mode() == RobotMode::Balancing);
  assert(state.motorsEnabled());
  assert(state.uprightAngleDegrees() == 0.7f);
}

static void test_start_balancing_at_is_ignored_when_not_disarmed() {
  RobotState state;
  state.configure(30.0f, 4.0f, 100, 150);

  assert(state.startBalancingAt(0.7f));
  assert(!state.startBalancingAt(2.0f));

  assert(state.uprightAngleDegrees() == 0.7f);
}
```

Add the calls in `main()`:

```cpp
  test_start_balancing_at_sets_absolute_upright_angle();
  test_start_balancing_at_is_ignored_when_not_disarmed();
```

- [ ] **Step 2: Verify the RobotState tests fail**

Run:

```bash
make test_robot_state
```

Expected: FAIL because `RobotState::startBalancingAt` is not declared.

- [ ] **Step 3: Implement direct balancing entry**

Modify `SelfBalanceRobot/RobotState.h`:

```cpp
  bool startBalancingAt(float uprightAngleDegrees);
```

Place it after `void update(...)`.

Modify `SelfBalanceRobot/RobotState.cpp` after `update(...)`:

```cpp
bool RobotState::startBalancingAt(float uprightAngleDegrees) {
  if (mode_ != RobotMode::Disarmed) {
    return false;
  }

  uprightAngleDegrees_ = uprightAngleDegrees;
  calibrationInitialAngleDegrees_ = uprightAngleDegrees;
  calibrationMinAngleDegrees_ = uprightAngleDegrees;
  calibrationMaxAngleDegrees_ = uprightAngleDegrees;
  calibrationSumDegrees_ = uprightAngleDegrees;
  calibrationSamples_ = 1;
  mode_ = RobotMode::Balancing;
  return true;
}
```

- [ ] **Step 4: Verify RobotState tests pass**

Run:

```bash
make test_robot_state
```

Expected: PASS with `test_robot_state PASS`.

- [ ] **Step 5: Commit**

```bash
git add SelfBalanceRobot/RobotState.* tests/native/test_robot_state.cpp
git commit -m "feat: allow direct balance entry at persisted target"
```

---

### Task 5: Sketch Integration And Config

**Files:**
- Modify: `SelfBalanceRobot/config.h`
- Modify: `SelfBalanceRobot/SelfBalanceRobot.ino`

- [ ] **Step 1: Add configuration constants**

Modify `SelfBalanceRobot/config.h` inside `namespace Config` after `MaxStartupUprightAngleDegrees`:

```cpp
  constexpr bool EnableAutoArm = true;
  constexpr uint16_t BalancePointEepromAddress = 0;
  constexpr float AutoArmDefaultBalancePointDegrees = 0.70f;
  constexpr float MinPersistedBalancePointDegrees = -12.0f;
  constexpr float MaxPersistedBalancePointDegrees = 12.0f;
  constexpr float AutoArmAngleWindowDegrees = 3.0f;
  constexpr float AutoArmMaxRateDegPerSec = 6.0f;
  constexpr unsigned long AutoArmStillMillis = 900UL;
  constexpr unsigned long AutoArmStopCooldownMillis = 3000UL;

  constexpr unsigned long BalancePointLearningSettleMillis = 1500UL;
  constexpr unsigned long BalancePointLearningStableMillis = 1000UL;
  constexpr unsigned long BalancePointMinWriteIntervalMillis = 30000UL;
  constexpr float BalancePointLearningMaxAngleErrorDegrees = 1.0f;
  constexpr float BalancePointLearningMaxRateDegPerSec = 5.0f;
  constexpr int16_t BalancePointLearningMaxMotorCommand = 45;
  constexpr float BalancePointLearningAlpha = 0.25f;
```

- [ ] **Step 2: Wire the new modules into the sketch**

Modify the includes at the top of `SelfBalanceRobot/SelfBalanceRobot.ino`:

```cpp
#include "AutoArmController.h"
#include "BalancePointLearner.h"
#include "BalancePointStore.h"
#include "EepromByteStorage.h"
```

Add globals after `RobotState robotState;`:

```cpp
EepromByteStorage eepromStorage;
BalancePointStore balancePointStore(eepromStorage,
                                    Config::BalancePointEepromAddress);
AutoArmController autoArm;
BalancePointLearner balancePointLearner;
```

Add globals after `unsigned long balancingStartMillis = 0;`:

```cpp
bool balanceSessionUsesPersistedPoint = false;
float activeBalancePointDegrees = Config::AutoArmDefaultBalancePointDegrees;
```

Add forward declarations:

```cpp
void configureAutoArmAndLearning();
void printBalancePointStatus(bool loaded);
void handleAutoArm(const SensorFrame& frame, RobotMode modeBeforeAutoArm);
void updateBalancePointLearning(const SensorFrame& frame,
                                float baseTargetDegrees,
                                int16_t balanceOutput,
                                unsigned long nowMillis);
```

Add the target helper as a file-scope function because `.ino` helpers are free functions:

```cpp
float baseBalanceTargetDegrees();
```

- [ ] **Step 3: Configure EEPROM, auto-arm, and learner in setup**

In `setup()`, after the balance controller setup and before `robotState.configure(...)`, add:

```cpp
  configureAutoArmAndLearning();
```

Add this function below `setup()`:

```cpp
void configureAutoArmAndLearning() {
  balancePointStore.configure(Config::MinPersistedBalancePointDegrees,
                              Config::MaxPersistedBalancePointDegrees);
  const bool loaded =
      balancePointStore.begin(Config::AutoArmDefaultBalancePointDegrees);
  activeBalancePointDegrees = balancePointStore.balancePointDegrees();

  autoArm.configure(Config::AutoArmAngleWindowDegrees,
                    Config::AutoArmMaxRateDegPerSec,
                    Config::AutoArmStillMillis);
  autoArm.setTargetBalancePoint(activeBalancePointDegrees);

  balancePointLearner.configure(Config::BalancePointLearningSettleMillis,
                                Config::BalancePointLearningStableMillis,
                                Config::BalancePointMinWriteIntervalMillis,
                                Config::BalancePointLearningMaxAngleErrorDegrees,
                                Config::BalancePointLearningMaxRateDegPerSec,
                                Config::BalancePointLearningMaxMotorCommand,
                                Config::BalancePointLearningAlpha);
  printBalancePointStatus(loaded);
}

void printBalancePointStatus(bool loaded) {
  if (!Config::EnableDebugSerial) {
    return;
  }

  Serial.print(F("balance-point "));
  Serial.print(loaded ? F("loaded=") : F("default="));
  Serial.print(balancePointStore.balancePointDegrees());
  Serial.print(F(" writes="));
  Serial.println(balancePointStore.writeCounter());
}
```

- [ ] **Step 4: Add auto-arm transition logic**

In `loop()`, keep `robotState.update(frame, command);`, then add auto-arm handling before reading `currentMode`:

```cpp
  const RobotMode previousMode = robotState.mode();
  robotState.update(frame, command);
  const bool stopWasHandled = command.stop;
  if (stopWasHandled) {
    command.stop = false;
    autoArm.suppressUntil(nowMillis, Config::AutoArmStopCooldownMillis);
    balanceSessionUsesPersistedPoint = false;
  }

  handleAutoArm(frame, previousMode);
  const RobotMode currentMode = robotState.mode();
```

Add this function:

```cpp
void handleAutoArm(const SensorFrame& frame, RobotMode modeBeforeAutoArm) {
  if (!Config::EnableAutoArm) {
    return;
  }
  if (modeBeforeAutoArm != RobotMode::Disarmed ||
      robotState.mode() != RobotMode::Disarmed ||
      command.arm || command.stop ||
      static_cast<long>(motorTestUntilMillis - frame.nowMillis) > 0) {
    autoArm.reset();
    return;
  }

  if (!autoArm.update(frame)) {
    return;
  }

  activeBalancePointDegrees = balancePointStore.balancePointDegrees();
  if (robotState.startBalancingAt(activeBalancePointDegrees)) {
    balanceSessionUsesPersistedPoint = true;
    if (Config::EnableDebugSerial) {
      Serial.print(F("auto-arm balancePoint="));
      Serial.println(activeBalancePointDegrees);
    }
  }
}
```

- [ ] **Step 5: Track session target and reset learner on balance start**

In the existing block:

```cpp
  if (previousMode != currentMode && currentMode == RobotMode::Balancing) {
```

replace the block body with:

```cpp
    motors.resetTravel();
    lastWheelFeedback = motors.updateFeedback();
    balance.reset();
    lastBalanceOutput = 0;
    balancingStartMillis = nowMillis;
    if (!balanceSessionUsesPersistedPoint) {
      activeBalancePointDegrees =
          robotState.uprightAngleDegrees() + currentTrimDegrees;
    }
    balancePointLearner.reset(balancePointStore.balancePointDegrees(),
                              nowMillis);
```

- [ ] **Step 6: Use persisted target semantics while balancing**

Inside `if (robotState.motorsEnabled())`, replace:

```cpp
    const float finalTarget = robotState.uprightAngleDegrees() +
                              currentTrimDegrees + speedCorrection +
                              lastTravelHoldTargetCorrection;
```

with:

```cpp
    const float baseTarget = baseBalanceTargetDegrees();
    const float finalTarget = baseTarget + speedCorrection +
                              lastTravelHoldTargetCorrection;
```

Replace the ramp calculation with:

```cpp
    const float rampStartTarget = balanceSessionUsesPersistedPoint
                                      ? baseTarget
                                      : robotState.uprightAngleDegrees();
    if (elapsed < rampMs) {
      const float rampFraction = static_cast<float>(elapsed) /
                                 static_cast<float>(rampMs);
      lastTargetAngle = rampStartTarget +
                        rampFraction * (finalTarget - rampStartTarget);
    } else {
      lastTargetAngle = finalTarget;
    }
```

Add this helper:

```cpp
float baseBalanceTargetDegrees() {
  if (balanceSessionUsesPersistedPoint) {
    return activeBalancePointDegrees;
  }
  return robotState.uprightAngleDegrees() + currentTrimDegrees;
}
```

- [ ] **Step 7: Persist stable learned balance points**

After `lastBalanceOutput = balanceOutput;`, add:

```cpp
    updateBalancePointLearning(frame, baseTarget, balanceOutput, nowMillis);
```

Add this function:

```cpp
void updateBalancePointLearning(const SensorFrame& frame,
                                float baseTargetDegrees,
                                int16_t balanceOutput,
                                unsigned long nowMillis) {
  const BalanceLearningResult result =
      balancePointLearner.update(frame, baseTargetDegrees, balanceOutput,
                                 nowMillis);
  if (!result.shouldSave) {
    return;
  }

  if (!balancePointStore.saveBalancePoint(result.balancePointDegrees)) {
    return;
  }

  activeBalancePointDegrees = balancePointStore.balancePointDegrees();
  autoArm.setTargetBalancePoint(activeBalancePointDegrees);
  if (Config::EnableDebugSerial) {
    Serial.print(F("balance-point saved="));
    Serial.print(activeBalancePointDegrees);
    Serial.print(F(" writes="));
    Serial.println(balancePointStore.writeCounter());
  }
}
```

- [ ] **Step 8: Suppress auto-arm on serial STOP**

In `readUsbCommand`, inside the `stop` branch after `motorTestUntilMillis = 0;`, add:

```cpp
          autoArm.suppressUntil(nowMillis, Config::AutoArmStopCooldownMillis);
          balanceSessionUsesPersistedPoint = false;
```

- [ ] **Step 9: Verify native tests and Arduino compile**

Run:

```bash
make
arduino-cli compile --fqbn arduino:avr:mega SelfBalanceRobot
```

Expected: all native tests pass and Arduino compile succeeds.

- [ ] **Step 10: Commit**

```bash
git add SelfBalanceRobot/config.h SelfBalanceRobot/SelfBalanceRobot.ino
git commit -m "feat: integrate auto-arm and balance point learning"
```

---

### Task 6: Bring-Up Documentation

**Files:**
- Modify: `docs/bring-up.md`

- [ ] **Step 1: Update the serial command section**

Add this paragraph after the serial command table:

```markdown
Auto-arm is enabled by default. If EEPROM contains a valid learned balance point, the robot can enter balancing without a serial `ARM` when it is held nearly still near that angle. `STOP` still disarms immediately and starts a short auto-arm cooldown.
```

- [ ] **Step 2: Add a cable-free auto-arm section**

Add this section after `## Release Technique`:

```markdown
## Cable-Free Auto-Arm

1. Power the robot without the USB cable attached.
2. Hold it near the known balance point and as still as possible.
3. Wait for auto-arm to enter balancing; the LED becomes solid when balancing starts.
4. Keep holding for another 500 ms-1 s so the controller settles before release.
5. If it re-arms too aggressively after a catch, send `STOP` while connected or power-cycle and increase `AutoArmStopCooldownMillis`.

The learned balance point is stored in EEPROM. During stable balancing, firmware may update it slowly using `BalancePointLearningAlpha`, but it will not write continuously while the robot is unstable.
```

- [ ] **Step 3: Document tuning constants**

In the tuning baseline block, add:

```markdown
AutoArmDefaultBalancePointDegrees = 0.70
AutoArmAngleWindowDegrees = 3.0
AutoArmMaxRateDegPerSec = 6.0
AutoArmStillMillis = 900
BalancePointLearningAlpha = 0.25
```

- [ ] **Step 4: Verify docs have no stale contradiction**

Run:

```bash
rg -n "requires an arm command|must send ARM|Auto-arm|auto-arm" README.md docs/bring-up.md
```

Expected: `docs/bring-up.md` describes auto-arm clearly. If `README.md` still says arming is required, update that sentence to mention serial `ARM` or auto-arm.

- [ ] **Step 5: Commit**

```bash
git add docs/bring-up.md README.md
git commit -m "docs: document cable-free auto-arm"
```

---

### Final Verification

- [ ] Run all native tests:

```bash
make
```

Expected:

```text
test_balance_controller PASS
test_robot_state PASS
test_sensors PASS
test_auto_arm_controller PASS
test_balance_point_store PASS
test_balance_point_learner PASS
```

- [ ] Compile the Arduino sketch:

```bash
arduino-cli compile --fqbn arduino:avr:mega SelfBalanceRobot
```

Expected: compile succeeds and reports flash/RAM usage.

- [ ] Check changed files:

```bash
git status --short
```

Expected: only intentional files are modified or untracked.

---

## Self-Review

Spec coverage:
- Persisted absolute balance angle: Task 2 and Task 5.
- Auto-arm near stored point with stillness gates: Task 1 and Task 5.
- STOP cooldown: Task 1 and Task 5.
- Stable learning with smoothing and EEPROM wear protection: Task 3 and Task 5.
- Manual ARM compatibility: Task 5 keeps the existing calibration path and Task 4 preserves existing `RobotState` tests.
- Tests and Arduino compile: every task has targeted test commands and final verification covers both native and Arduino builds.

Placeholder scan: clean.

Type consistency: `AutoArmController`, `BalancePointStore`, `EepromByteStorage`, `BalancePointLearner`, `BalanceLearningResult`, and `RobotState::startBalancingAt` are named consistently across tests, implementations, and sketch integration.
