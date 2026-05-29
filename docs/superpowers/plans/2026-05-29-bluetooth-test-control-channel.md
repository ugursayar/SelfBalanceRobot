# Bluetooth Test Control Channel Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore Bluetooth as a cable-free test/control console using the same command parser as USB serial.

**Architecture:** Add a pure `CommandParser` for newline text commands, a small `CommandReader` for per-port `Stream` buffering, and keep hardware side effects in `SelfBalanceRobot.ino`. Add explicit balance-point clear/set support in `BalancePointStore`, then wire USB `Serial` and Bluetooth `Serial1` through the shared command path.

**Tech Stack:** Arduino C++11, Makeblock `MeMegaPi`, AVR `Serial`/`Serial1`, native `g++` tests under `tests/native`.

---

## File Structure

- Create `SelfBalanceRobot/CommandParser.h`
  - Defines `ParsedCommandAction` and `ParsedCommand`.
  - Exposes `CommandParser::parse(char* line)`.
- Create `SelfBalanceRobot/CommandParser.cpp`
  - Lowercases and tokenizes one mutable command line.
  - Parses existing commands plus `BP?`, `BP SET`, `BP CLEAR`, `AUTO`, `LEARN`, `STATUS`, and `TELEM`.
- Create `SelfBalanceRobot/CommandReader.h`
  - Owns one input buffer per serial port.
  - Reads complete newline-terminated commands from a `Stream`.
- Create `SelfBalanceRobot/CommandReader.cpp`
  - Returns one parsed command at a time without sharing state across ports.
- Modify `SelfBalanceRobot/BalancePointStore.h`
  - Add `void clearBalancePoint(float fallbackDegrees);`.
- Modify `SelfBalanceRobot/BalancePointStore.cpp`
  - Invalidate both EEPROM slots and reset in-memory state to fallback/default.
- Modify `SelfBalanceRobot/config.h`
  - Add Bluetooth test-control constants.
- Modify `SelfBalanceRobot/SelfBalanceRobot.ino`
  - Replace inline USB parsing with `CommandReader`.
  - Start `Serial1` when Bluetooth test control is enabled.
  - Add runtime `AUTO`, `LEARN`, and Bluetooth telemetry flags.
  - Apply command side effects and send acknowledgements on the issuing port.
- Modify `tests/native/Makefile`
  - Build new parser and reader tests.
- Create `tests/native/test_command_parser.cpp`
  - Native tests for command grammar.
- Create `tests/native/test_command_reader.cpp`
  - Native tests for per-port buffering and newline behavior.
- Modify `tests/native/test_balance_point_store.cpp`
  - Add tests for clearing persisted records.
- Modify `docs/bring-up.md`
  - Document Bluetooth command flow and cable-free test sequence.

---

### Task 1: Add the Shared Command Parser

**Files:**
- Create: `SelfBalanceRobot/CommandParser.h`
- Create: `SelfBalanceRobot/CommandParser.cpp`
- Create: `tests/native/test_command_parser.cpp`
- Modify: `tests/native/Makefile`

- [ ] **Step 1: Write the failing parser tests**

Create `tests/native/test_command_parser.cpp` with tests shaped like this:

```cpp
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "../../SelfBalanceRobot/CommandParser.h"

static ParsedCommand parse(const char* text) {
  char line[48];
  std::strncpy(line, text, sizeof(line));
  line[sizeof(line) - 1] = '\0';
  CommandParser parser;
  return parser.parse(line);
}

static void test_existing_commands_are_case_insensitive() {
  assert(parse("ARM").action == ParsedCommandAction::Arm);
  assert(parse("stop").action == ParsedCommandAction::Stop);
  assert(parse("M+").action == ParsedCommandAction::MotorBackward);
  assert(parse("m-").action == ParsedCommandAction::MotorForward);
}

static void test_pid_and_trim_parse_numeric_arguments() {
  ParsedCommand pid = parse("PID 42 0 1.25");
  assert(pid.action == ParsedCommandAction::SetPid);
  assert(pid.first == 42.0f);
  assert(pid.second == 0.0f);
  assert(pid.third == 1.25f);

  ParsedCommand trim = parse("trim -2.5");
  assert(trim.action == ParsedCommandAction::SetTrim);
  assert(trim.first == -2.5f);
}

static void test_balance_point_commands_parse() {
  assert(parse("BP?").action == ParsedCommandAction::BalancePointQuery);
  assert(parse("bp clear").action == ParsedCommandAction::BalancePointClear);

  ParsedCommand explicitSet = parse("BP SET 0.85");
  assert(explicitSet.action == ParsedCommandAction::BalancePointSet);
  assert(explicitSet.first == 0.85f);

  ParsedCommand shortSet = parse("bp -1.20");
  assert(shortSet.action == ParsedCommandAction::BalancePointSet);
  assert(shortSet.first == -1.20f);
}

static void test_runtime_switch_commands_parse() {
  assert(parse("auto on").action == ParsedCommandAction::AutoOn);
  assert(parse("AUTO OFF").action == ParsedCommandAction::AutoOff);
  assert(parse("learn on").action == ParsedCommandAction::LearnOn);
  assert(parse("LEARN OFF").action == ParsedCommandAction::LearnOff);
  assert(parse("status").action == ParsedCommandAction::Status);
  assert(parse("telem on").action == ParsedCommandAction::TelemetryOn);
  assert(parse("TELEM OFF").action == ParsedCommandAction::TelemetryOff);
}

static void test_malformed_commands_are_invalid() {
  assert(parse("").action == ParsedCommandAction::None);
  assert(parse("pid 42 0").action == ParsedCommandAction::Invalid);
  assert(parse("pid 42 nope 1").action == ParsedCommandAction::Invalid);
  assert(parse("trim").action == ParsedCommandAction::Invalid);
  assert(parse("bp set").action == ParsedCommandAction::Invalid);
  assert(parse("bp not-a-number").action == ParsedCommandAction::Invalid);
  assert(parse("bp clear 1").action == ParsedCommandAction::Invalid);
  assert(parse("auto maybe").action == ParsedCommandAction::Invalid);
  assert(parse("hello").action == ParsedCommandAction::Invalid);
}

int main() {
  test_existing_commands_are_case_insensitive();
  test_pid_and_trim_parse_numeric_arguments();
  test_balance_point_commands_parse();
  test_runtime_switch_commands_parse();
  test_malformed_commands_are_invalid();

  std::cout << "test_command_parser PASS\n";
  return EXIT_SUCCESS;
}
```

Update `tests/native/Makefile` so the target exists and participates in `all`:

```make
.PHONY: all test_balance_controller test_robot_state test_sensors test_auto_arm_controller test_balance_point_store test_balance_point_learner test_command_parser clean

all: test_balance_controller test_robot_state test_sensors test_auto_arm_controller test_balance_point_store test_balance_point_learner test_command_parser

$(BUILD_DIR)/test_command_parser: test_command_parser.cpp $(ROOT)/SelfBalanceRobot/CommandParser.cpp $(ROOT)/SelfBalanceRobot/CommandParser.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(ROOT)/SelfBalanceRobot test_command_parser.cpp $(ROOT)/SelfBalanceRobot/CommandParser.cpp -o $@

test_command_parser: $(BUILD_DIR)/test_command_parser
	./$(BUILD_DIR)/test_command_parser
```

- [ ] **Step 2: Run parser test to verify it fails**

Run:

```powershell
make test_command_parser
```

Expected: FAIL because `SelfBalanceRobot/CommandParser.h` does not exist.

- [ ] **Step 3: Add the parser public API**

Create `SelfBalanceRobot/CommandParser.h`:

```cpp
#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <stdint.h>

enum class ParsedCommandAction : uint8_t {
  None,
  Invalid,
  Arm,
  Stop,
  MotorBackward,
  MotorForward,
  SetPid,
  SetTrim,
  BalancePointQuery,
  BalancePointSet,
  BalancePointClear,
  AutoOn,
  AutoOff,
  LearnOn,
  LearnOff,
  Status,
  TelemetryOn,
  TelemetryOff
};

struct ParsedCommand {
  ParsedCommandAction action = ParsedCommandAction::None;
  float first = 0.0f;
  float second = 0.0f;
  float third = 0.0f;
  unsigned long receivedMillis = 0;
};

class CommandParser {
public:
  ParsedCommand parse(char* line) const;

private:
  void normalize(char* line) const;
  bool parseFloatToken(const char* text, float& value) const;
  ParsedCommand invalid() const;
};

#endif
```

- [ ] **Step 4: Add the minimal parser implementation**

Create `SelfBalanceRobot/CommandParser.cpp`:

```cpp
#include "CommandParser.h"

#include <stdlib.h>
#include <string.h>

ParsedCommand CommandParser::parse(char* line) const {
  ParsedCommand command;
  if (line == nullptr) {
    return invalid();
  }

  normalize(line);
  char* token = strtok(line, " ");
  if (token == nullptr) {
    return command;
  }

  if (strcmp(token, "arm") == 0) {
    command.action = ParsedCommandAction::Arm;
    return command;
  }
  if (strcmp(token, "stop") == 0) {
    command.action = ParsedCommandAction::Stop;
    return command;
  }
  if (strcmp(token, "m+") == 0) {
    command.action = ParsedCommandAction::MotorBackward;
    return command;
  }
  if (strcmp(token, "m-") == 0) {
    command.action = ParsedCommandAction::MotorForward;
    return command;
  }
  if (strcmp(token, "status") == 0) {
    command.action = ParsedCommandAction::Status;
    return command;
  }
  if (strcmp(token, "bp?") == 0) {
    command.action = ParsedCommandAction::BalancePointQuery;
    return command;
  }

  if (strcmp(token, "trim") == 0) {
    char* value = strtok(nullptr, " ");
    if (value == nullptr || strtok(nullptr, " ") != nullptr ||
        !parseFloatToken(value, command.first)) {
      return invalid();
    }
    command.action = ParsedCommandAction::SetTrim;
    return command;
  }

  if (strcmp(token, "pid") == 0) {
    char* kp = strtok(nullptr, " ");
    char* ki = strtok(nullptr, " ");
    char* kd = strtok(nullptr, " ");
    if (kp == nullptr || ki == nullptr || kd == nullptr ||
        strtok(nullptr, " ") != nullptr ||
        !parseFloatToken(kp, command.first) ||
        !parseFloatToken(ki, command.second) ||
        !parseFloatToken(kd, command.third)) {
      return invalid();
    }
    command.action = ParsedCommandAction::SetPid;
    return command;
  }

  if (strcmp(token, "bp") == 0) {
    char* subcommand = strtok(nullptr, " ");
    if (subcommand == nullptr) {
      return invalid();
    }
    if (strcmp(subcommand, "clear") == 0) {
      if (strtok(nullptr, " ") != nullptr) {
        return invalid();
      }
      command.action = ParsedCommandAction::BalancePointClear;
      return command;
    }
    if (strcmp(subcommand, "set") == 0) {
      char* value = strtok(nullptr, " ");
      if (value == nullptr || strtok(nullptr, " ") != nullptr ||
          !parseFloatToken(value, command.first)) {
        return invalid();
      }
      command.action = ParsedCommandAction::BalancePointSet;
      return command;
    }
    if (strtok(nullptr, " ") != nullptr ||
        !parseFloatToken(subcommand, command.first)) {
      return invalid();
    }
    command.action = ParsedCommandAction::BalancePointSet;
    return command;
  }

  if (strcmp(token, "auto") == 0 || strcmp(token, "learn") == 0 ||
      strcmp(token, "telem") == 0) {
    const bool isAuto = strcmp(token, "auto") == 0;
    const bool isLearn = strcmp(token, "learn") == 0;
    char* value = strtok(nullptr, " ");
    if (value == nullptr || strtok(nullptr, " ") != nullptr) {
      return invalid();
    }
    const bool on = strcmp(value, "on") == 0;
    const bool off = strcmp(value, "off") == 0;
    if (!on && !off) {
      return invalid();
    }
    if (isAuto) {
      command.action = on ? ParsedCommandAction::AutoOn
                          : ParsedCommandAction::AutoOff;
    } else if (isLearn) {
      command.action = on ? ParsedCommandAction::LearnOn
                          : ParsedCommandAction::LearnOff;
    } else {
      command.action = on ? ParsedCommandAction::TelemetryOn
                          : ParsedCommandAction::TelemetryOff;
    }
    return command;
  }

  return invalid();
}

void CommandParser::normalize(char* line) const {
  char* write = line;
  bool lastWasSpace = true;
  for (char* read = line; *read != '\0'; ++read) {
    char c = *read;
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c + ('a' - 'A'));
    }
    if (c == '\t') {
      c = ' ';
    }
    if (c == ' ') {
      if (!lastWasSpace) {
        *write++ = c;
      }
      lastWasSpace = true;
    } else {
      *write++ = c;
      lastWasSpace = false;
    }
  }
  if (write > line && *(write - 1) == ' ') {
    --write;
  }
  *write = '\0';
}

bool CommandParser::parseFloatToken(const char* text, float& value) const {
  char* end = nullptr;
  const double parsed = strtod(text, &end);
  if (end == text || *end != '\0') {
    return false;
  }
  value = static_cast<float>(parsed);
  return true;
}

ParsedCommand CommandParser::invalid() const {
  ParsedCommand command;
  command.action = ParsedCommandAction::Invalid;
  return command;
}
```

- [ ] **Step 5: Run parser test to verify it passes**

Run:

```powershell
make test_command_parser
```

Expected: PASS and prints `test_command_parser PASS`.

- [ ] **Step 6: Commit parser**

Run:

```powershell
git add SelfBalanceRobot/CommandParser.* tests/native/test_command_parser.cpp tests/native/Makefile
git commit -m "feat: add shared command parser"
```

---

### Task 2: Add Per-Port Command Readers

**Files:**
- Create: `SelfBalanceRobot/CommandReader.h`
- Create: `SelfBalanceRobot/CommandReader.cpp`
- Create: `tests/native/test_command_reader.cpp`
- Modify: `tests/native/Makefile`

- [ ] **Step 1: Write the failing reader tests**

Create `tests/native/test_command_reader.cpp`:

```cpp
#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "../../SelfBalanceRobot/CommandReader.h"

class FakeStream : public Stream {
public:
  explicit FakeStream(const char* text) : text_(text), index_(0) {}

  int available() {
    return text_[index_] == '\0' ? 0 : 1;
  }

  int read() {
    if (text_[index_] == '\0') {
      return -1;
    }
    return text_[index_++];
  }

private:
  const char* text_;
  size_t index_;
};

static void test_reader_waits_for_newline() {
  FakeStream stream("arm");
  CommandReader reader;
  reader.begin(stream);

  ParsedCommand command;
  assert(!reader.readCommand(command, 10));
}

static void test_reader_parses_complete_line_with_timestamp() {
  FakeStream stream("ARM\n");
  CommandReader reader;
  reader.begin(stream);

  ParsedCommand command;
  assert(reader.readCommand(command, 123));
  assert(command.action == ParsedCommandAction::Arm);
  assert(command.receivedMillis == 123);
}

static void test_reader_returns_one_command_per_call() {
  FakeStream stream("arm\nstop\n");
  CommandReader reader;
  reader.begin(stream);

  ParsedCommand first;
  ParsedCommand second;
  assert(reader.readCommand(first, 1));
  assert(first.action == ParsedCommandAction::Arm);
  assert(reader.readCommand(second, 2));
  assert(second.action == ParsedCommandAction::Stop);
  assert(second.receivedMillis == 2);
}

static void test_readers_keep_independent_buffers() {
  FakeStream usb("ar");
  FakeStream bluetooth("stop\n");
  CommandReader usbReader;
  CommandReader bluetoothReader;
  usbReader.begin(usb);
  bluetoothReader.begin(bluetooth);

  ParsedCommand command;
  assert(!usbReader.readCommand(command, 1));
  assert(bluetoothReader.readCommand(command, 2));
  assert(command.action == ParsedCommandAction::Stop);
}

int main() {
  test_reader_waits_for_newline();
  test_reader_parses_complete_line_with_timestamp();
  test_reader_returns_one_command_per_call();
  test_readers_keep_independent_buffers();

  std::cout << "test_command_reader PASS\n";
  return EXIT_SUCCESS;
}
```

Update `tests/native/Makefile`:

```make
.PHONY: all test_balance_controller test_robot_state test_sensors test_auto_arm_controller test_balance_point_store test_balance_point_learner test_command_parser test_command_reader clean

all: test_balance_controller test_robot_state test_sensors test_auto_arm_controller test_balance_point_store test_balance_point_learner test_command_parser test_command_reader

$(BUILD_DIR)/test_command_reader: test_command_reader.cpp $(ROOT)/SelfBalanceRobot/CommandReader.cpp $(ROOT)/SelfBalanceRobot/CommandReader.h $(ROOT)/SelfBalanceRobot/CommandParser.cpp $(ROOT)/SelfBalanceRobot/CommandParser.h $(BUILD_DIR)/Arduino.h | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -I$(BUILD_DIR) -I$(ROOT)/SelfBalanceRobot test_command_reader.cpp $(ROOT)/SelfBalanceRobot/CommandReader.cpp $(ROOT)/SelfBalanceRobot/CommandParser.cpp -o $@

test_command_reader: $(BUILD_DIR)/test_command_reader
	./$(BUILD_DIR)/test_command_reader
```

- [ ] **Step 2: Run reader test to verify it fails**

Run:

```powershell
make test_command_reader
```

Expected: FAIL because `SelfBalanceRobot/CommandReader.h` does not exist.

- [ ] **Step 3: Add the reader public API**

Create `SelfBalanceRobot/CommandReader.h`:

```cpp
#ifndef COMMAND_READER_H
#define COMMAND_READER_H

#include <Arduino.h>
#include <stdint.h>

#include "CommandParser.h"

class CommandReader {
public:
  CommandReader();

  void begin(Stream& stream);
  bool readCommand(ParsedCommand& command, unsigned long nowMillis);

private:
  static const uint8_t kBufferSize = 48;

  Stream* stream_;
  CommandParser parser_;
  char buffer_[kBufferSize];
  uint8_t length_;
  bool overflow_;
};

#endif
```

- [ ] **Step 4: Add the reader implementation**

Create `SelfBalanceRobot/CommandReader.cpp`:

```cpp
#include "CommandReader.h"

CommandReader::CommandReader()
    : stream_(nullptr), length_(0), overflow_(false) {
  buffer_[0] = '\0';
}

void CommandReader::begin(Stream& stream) {
  stream_ = &stream;
  length_ = 0;
  overflow_ = false;
  buffer_[0] = '\0';
}

bool CommandReader::readCommand(ParsedCommand& command,
                                unsigned long nowMillis) {
  if (stream_ == nullptr) {
    return false;
  }

  while (stream_->available() > 0) {
    const int raw = stream_->read();
    if (raw < 0) {
      return false;
    }
    const char incoming = static_cast<char>(raw);
    if (incoming == '\r' || incoming == '\n') {
      if (length_ == 0 && !overflow_) {
        continue;
      }
      buffer_[length_] = '\0';
      command = overflow_ ? ParsedCommand() : parser_.parse(buffer_);
      if (overflow_) {
        command.action = ParsedCommandAction::Invalid;
      }
      command.receivedMillis = nowMillis;
      length_ = 0;
      overflow_ = false;
      buffer_[0] = '\0';
      return true;
    }

    if (length_ < kBufferSize - 1) {
      buffer_[length_++] = incoming;
    } else {
      overflow_ = true;
    }
  }

  return false;
}
```

- [ ] **Step 5: Run reader test to verify it passes**

Run:

```powershell
make test_command_reader
```

Expected: PASS and prints `test_command_reader PASS`.

- [ ] **Step 6: Commit reader**

Run:

```powershell
git add SelfBalanceRobot/CommandReader.* tests/native/test_command_reader.cpp tests/native/Makefile
git commit -m "feat: add serial command reader"
```

---

### Task 3: Add Balance-Point Clear Support

**Files:**
- Modify: `SelfBalanceRobot/BalancePointStore.h`
- Modify: `SelfBalanceRobot/BalancePointStore.cpp`
- Modify: `tests/native/test_balance_point_store.cpp`

- [ ] **Step 1: Write the failing balance-point clear test**

Add this test to `tests/native/test_balance_point_store.cpp`, before `main()`:

```cpp
static void test_clear_invalidates_records_and_uses_fallback() {
  FakeStorage storage;
  BalancePointStore writer = storeFor(storage);
  assert(!writer.begin(0.7f));
  assert(writer.saveBalancePoint(1.25f));
  assert(writer.saveBalancePoint(2.5f));

  writer.clearBalancePoint(0.7f);
  assert(!writer.hasStoredBalancePoint());
  assert(writer.balancePointDegrees() == 0.7f);
  assert(writer.writeCounter() == 0);

  BalancePointStore reader = storeFor(storage);
  assert(!reader.begin(0.7f));
  assert(!reader.hasStoredBalancePoint());
  assert(reader.balancePointDegrees() == 0.7f);
  assert(reader.writeCounter() == 0);
}
```

Call it from `main()`:

```cpp
  test_clear_invalidates_records_and_uses_fallback();
```

- [ ] **Step 2: Run store test to verify it fails**

Run:

```powershell
make test_balance_point_store
```

Expected: FAIL because `BalancePointStore::clearBalancePoint` is not declared.

- [ ] **Step 3: Add clear method to the header**

In `SelfBalanceRobot/BalancePointStore.h`, add the public method after `saveBalancePoint`:

```cpp
  bool saveBalancePoint(float angleDegrees);
  void clearBalancePoint(float fallbackDegrees);
```

- [ ] **Step 4: Add clear method implementation**

In `SelfBalanceRobot/BalancePointStore.cpp`, add after `saveBalancePoint`:

```cpp
void BalancePointStore::clearBalancePoint(float fallbackDegrees) {
  for (uint8_t slot = 0; slot < kSlotCount; ++slot) {
    writeByte(slotOffset(slot), 0xFF);
  }
  balancePointDegrees_ = fallbackDegrees;
  writeCounter_ = 0;
  hasStoredBalancePoint_ = false;
  activeSlot_ = kNoActiveSlot;
}
```

- [ ] **Step 5: Run store test to verify it passes**

Run:

```powershell
make test_balance_point_store
```

Expected: PASS and prints `test_balance_point_store PASS`.

- [ ] **Step 6: Commit store clear**

Run:

```powershell
git add SelfBalanceRobot/BalancePointStore.* tests/native/test_balance_point_store.cpp
git commit -m "feat: allow clearing persisted balance point"
```

---

### Task 4: Wire Shared Commands Into USB and Bluetooth

**Files:**
- Modify: `SelfBalanceRobot/config.h`
- Modify: `SelfBalanceRobot/SelfBalanceRobot.ino`

- [ ] **Step 1: Add config constants**

In `SelfBalanceRobot/config.h`, add near `EnableDebugSerial`:

```cpp
  constexpr bool EnableBluetoothTestControl = true;
  constexpr unsigned long BluetoothBaud = 115200UL;
```

- [ ] **Step 2: Include command reader in the sketch**

At the top of `SelfBalanceRobot/SelfBalanceRobot.ino`, add:

```cpp
#include "CommandReader.h"
```

- [ ] **Step 3: Replace parser globals and prototypes**

Add these globals near the existing global objects:

```cpp
CommandReader usbCommandReader;
CommandReader bluetoothCommandReader;
bool runtimeAutoArmEnabled = Config::EnableAutoArm;
bool balancePointLearningEnabled = true;
bool bluetoothTelemetryEnabled = false;
```

Replace the old prototype:

```cpp
void readUsbCommand(unsigned long nowMillis);
```

with:

```cpp
void readCommands(unsigned long nowMillis);
void readCommandsFrom(CommandReader& reader, Stream& reply,
                      unsigned long nowMillis);
void applyParsedCommand(const ParsedCommand& parsed, Stream& reply);
void applyStopCommand(unsigned long nowMillis);
void clearPersistedBalancePoint(Stream& reply, unsigned long nowMillis);
void setPersistedBalancePoint(float angleDegrees, Stream& reply,
                              unsigned long nowMillis);
void printBalancePointStatusTo(Stream& out);
void printStatus(Stream& out, const SensorFrame& frame);
void printMode(Stream& out, RobotMode mode);
void printDebugTo(Stream& out, const SensorFrame& frame);
void printAutonomousEventToTelemetry(const __FlashStringHelper* label,
                                     float value);
```

Keep the existing `void printMode(RobotMode mode);` prototype out of the final sketch; callers will use the `Stream&` version.

- [ ] **Step 4: Initialize readers and Bluetooth**

In `setup()`, after `Serial.begin(115200);`, add:

```cpp
  usbCommandReader.begin(Serial);
  if (Config::EnableBluetoothTestControl) {
    Serial1.begin(Config::BluetoothBaud);
    bluetoothCommandReader.begin(Serial1);
  }
```

Change the ready line to mention Bluetooth:

```cpp
    Serial.println(F("SelfBalanceRobot balance-only ready. Send arm or stop."));
```

can remain unchanged for USB. Add this acknowledgement after the existing ready line:

```cpp
    if (Config::EnableBluetoothTestControl) {
      Serial.println(F("bluetooth-test-control serial1=115200"));
    }
```

- [ ] **Step 5: Replace command polling in `loop()`**

Replace:

```cpp
  readUsbCommand(nowMillis);
```

with:

```cpp
  readCommands(nowMillis);
```

- [ ] **Step 6: Gate auto-arm and learning**

In `handleAutoArm`, replace:

```cpp
  if (!Config::EnableAutoArm) {
```

with:

```cpp
  if (!runtimeAutoArmEnabled) {
```

In `updateBalancePointLearning`, add this at the top:

```cpp
  if (!balancePointLearningEnabled) {
    return;
  }
```

- [ ] **Step 7: Add shared command application helpers**

Delete the existing `readUsbCommand` function from `SelfBalanceRobot.ino`.

Add these functions in its place:

```cpp
void readCommands(unsigned long nowMillis) {
  readCommandsFrom(usbCommandReader, Serial, nowMillis);
  if (Config::EnableBluetoothTestControl) {
    readCommandsFrom(bluetoothCommandReader, Serial1, nowMillis);
  }
}

void readCommandsFrom(CommandReader& reader, Stream& reply,
                      unsigned long nowMillis) {
  ParsedCommand parsed;
  while (reader.readCommand(parsed, nowMillis)) {
    applyParsedCommand(parsed, reply);
  }
}

void applyParsedCommand(const ParsedCommand& parsed, Stream& reply) {
  switch (parsed.action) {
  case ParsedCommandAction::None:
    return;
  case ParsedCommandAction::Invalid:
    reply.println(F("error command"));
    return;
  case ParsedCommandAction::Arm:
    if (!command.stop) {
      command.arm = true;
      command.stop = false;
      command.receivedMillis = parsed.receivedMillis;
      reply.println(F("ok arm"));
    }
    return;
  case ParsedCommandAction::Stop:
    applyStopCommand(parsed.receivedMillis);
    reply.println(F("ok stop"));
    return;
  case ParsedCommandAction::MotorBackward:
    startMotorTest(Config::MotorTestCommand, parsed.receivedMillis);
    reply.println(F("ok m+"));
    return;
  case ParsedCommandAction::MotorForward:
    startMotorTest(-Config::MotorTestCommand, parsed.receivedMillis);
    reply.println(F("ok m-"));
    return;
  case ParsedCommandAction::SetTrim:
    applyRuntimeTrim(parsed.first);
    reply.print(F("ok trim="));
    reply.println(currentTrimDegrees);
    return;
  case ParsedCommandAction::SetPid:
    applyRuntimePid(parsed.first, parsed.second, parsed.third);
    reply.print(F("ok pid kp="));
    reply.print(currentKp);
    reply.print(F(" ki="));
    reply.print(currentKi);
    reply.print(F(" kd="));
    reply.println(currentKd);
    return;
  case ParsedCommandAction::BalancePointQuery:
    printBalancePointStatusTo(reply);
    return;
  case ParsedCommandAction::BalancePointSet:
    setPersistedBalancePoint(parsed.first, reply, parsed.receivedMillis);
    return;
  case ParsedCommandAction::BalancePointClear:
    clearPersistedBalancePoint(reply, parsed.receivedMillis);
    return;
  case ParsedCommandAction::AutoOn:
    runtimeAutoArmEnabled = Config::EnableAutoArm;
    reply.println(runtimeAutoArmEnabled ? F("ok auto=on")
                                        : F("ok auto=disabled-by-config"));
    return;
  case ParsedCommandAction::AutoOff:
    runtimeAutoArmEnabled = false;
    autoArm.suppressUntil(parsed.receivedMillis,
                          Config::AutoArmStopCooldownMillis);
    reply.println(F("ok auto=off"));
    return;
  case ParsedCommandAction::LearnOn:
    balancePointLearningEnabled = true;
    balancePointLearner.reset(balancePointStore.balancePointDegrees(),
                              parsed.receivedMillis);
    reply.println(F("ok learn=on"));
    return;
  case ParsedCommandAction::LearnOff:
    balancePointLearningEnabled = false;
    reply.println(F("ok learn=off"));
    return;
  case ParsedCommandAction::Status:
    printStatus(reply, lastFrame);
    return;
  case ParsedCommandAction::TelemetryOn:
    bluetoothTelemetryEnabled = true;
    reply.println(F("ok telem=on"));
    return;
  case ParsedCommandAction::TelemetryOff:
    bluetoothTelemetryEnabled = false;
    reply.println(F("ok telem=off"));
    return;
  }
}

void applyStopCommand(unsigned long nowMillis) {
  command.arm = false;
  command.stop = true;
  command.receivedMillis = nowMillis;
  motorTestUntilMillis = 0;
  autoArm.suppressUntil(nowMillis, Config::AutoArmStopCooldownMillis);
  balanceSessionUsesPersistedPoint = false;
}

void clearPersistedBalancePoint(Stream& reply, unsigned long nowMillis) {
  applyStopCommand(nowMillis);
  balancePointStore.clearBalancePoint(Config::AutoArmDefaultBalancePointDegrees);
  activeBalancePointDegrees = balancePointStore.balancePointDegrees();
  autoArm.setTargetBalancePoint(activeBalancePointDegrees);
  balancePointLearner.reset(activeBalancePointDegrees, nowMillis);
  reply.print(F("ok bp-clear default="));
  reply.println(activeBalancePointDegrees);
}

void setPersistedBalancePoint(float angleDegrees, Stream& reply,
                              unsigned long nowMillis) {
  if (!balancePointStore.saveBalancePoint(angleDegrees)) {
    reply.println(F("error bp-range"));
    return;
  }
  activeBalancePointDegrees = balancePointStore.balancePointDegrees();
  autoArm.setTargetBalancePoint(activeBalancePointDegrees);
  balancePointLearner.reset(activeBalancePointDegrees, nowMillis);
  reply.print(F("ok bp="));
  reply.print(activeBalancePointDegrees);
  reply.print(F(" writes="));
  reply.println(balancePointStore.writeCounter());
}
```

- [ ] **Step 8: Route mode and status printing through `Stream&`**

Replace the existing `printMode(RobotMode mode)` implementation with:

```cpp
void printMode(Stream& out, RobotMode mode) {
  switch (mode) {
  case RobotMode::Disarmed:
    out.print(F("disarmed"));
    break;
  case RobotMode::Calibrating:
    out.print(F("calibrating"));
    break;
  case RobotMode::Balancing:
    out.print(F("balancing"));
    break;
  case RobotMode::Fault:
    out.print(F("fault"));
    break;
  }
}
```

Update calls:

```cpp
  printMode(Serial, mode);
```

and:

```cpp
  printMode(out, robotState.mode());
```

Add balance-point and status helpers:

```cpp
void printBalancePointStatusTo(Stream& out) {
  out.print(F("balance-point active="));
  out.print(balancePointStore.balancePointDegrees());
  out.print(F(" stored="));
  out.print(balancePointStore.hasStoredBalancePoint() ? F("yes") : F("no"));
  out.print(F(" writes="));
  out.println(balancePointStore.writeCounter());
}

void printStatus(Stream& out, const SensorFrame& frame) {
  out.print(F("status mode="));
  printMode(out, robotState.mode());
  out.print(F(" angle="));
  out.print(frame.angleDegrees);
  out.print(F(" target="));
  out.print(lastTargetAngle);
  out.print(F(" trim="));
  out.print(currentTrimDegrees);
  out.print(F(" bp="));
  out.print(activeBalancePointDegrees);
  out.print(F(" stored="));
  out.print(balancePointStore.hasStoredBalancePoint() ? F("yes") : F("no"));
  out.print(F(" rate="));
  out.print(balance.lastMeasuredAngleRateDegreesPerSecond());
  out.print(F(" raw="));
  out.print(lastRawBalanceOutput);
  out.print(F(" balance="));
  out.print(lastBalanceOutput);
  out.print(F(" speed="));
  out.print(lastWheelFeedback.averageSpeedRpm);
  out.print(F(" pos="));
  out.print(lastWheelFeedback.averagePositionDegrees);
  out.print(F(" hold="));
  out.print(lastTravelHoldTargetCorrection);
  out.print(F(" left="));
  out.print(lastMotorOutput.left);
  out.print(F(" right="));
  out.print(lastMotorOutput.right);
  out.print(F(" kp="));
  out.print(currentKp);
  out.print(F(" ki="));
  out.print(currentKi);
  out.print(F(" kd="));
  out.println(currentKd);
}
```

- [ ] **Step 9: Share debug formatting with Bluetooth telemetry**

Replace the body of `printDebug` with:

```cpp
void printDebug(const SensorFrame& frame) {
  if (!Config::EnableDebugSerial && !bluetoothTelemetryEnabled) {
    return;
  }

  if (frame.nowMillis - lastDebugMillis < Config::DebugPeriodMillis) {
    return;
  }
  lastDebugMillis = frame.nowMillis;

  if (Config::EnableDebugSerial) {
    printDebugTo(Serial, frame);
  }
  if (Config::EnableBluetoothTestControl && bluetoothTelemetryEnabled) {
    printDebugTo(Serial1, frame);
  }
}
```

Move the existing print body into:

```cpp
void printDebugTo(Stream& out, const SensorFrame& frame) {
  out.print(F("mode="));
  printMode(out, robotState.mode());
  out.print(F(" loopUs="));
  out.print(lastLoopMicros);
  out.print(F(" angle="));
  out.print(frame.angleDegrees);
  out.print(F(" upright="));
  out.print(robotState.uprightAngleDegrees());
  out.print(F(" trim="));
  out.print(currentTrimDegrees);
  out.print(F(" target="));
  out.print(lastTargetAngle);
  out.print(F(" err="));
  out.print(balance.lastErrorDegrees());
  out.print(F(" rate="));
  out.print(balance.lastMeasuredAngleRateDegreesPerSecond());
  out.print(F(" raw="));
  out.print(lastRawBalanceOutput);
  out.print(F(" balance="));
  out.print(lastBalanceOutput);
  out.print(F(" speed="));
  out.print(lastWheelFeedback.averageSpeedRpm);
  out.print(F(" pos="));
  out.print(lastWheelFeedback.averagePositionDegrees);
  out.print(F(" hold="));
  out.print(lastTravelHoldTargetCorrection);
  out.print(F(" left="));
  out.print(lastMotorOutput.left);
  out.print(F(" right="));
  out.print(lastMotorOutput.right);
  out.print(F(" lpwm="));
  out.print(lastWheelFeedback.leftPwm);
  out.print(F(" rpwm="));
  out.print(lastWheelFeedback.rightPwm);
  out.print(F(" kp="));
  out.print(currentKp);
  out.print(F(" kd="));
  out.println(currentKd);
}
```

- [ ] **Step 10: Mirror autonomous events to Bluetooth telemetry**

In `handleAutoArm`, after the existing USB debug prints, add:

```cpp
      if (Config::EnableBluetoothTestControl && bluetoothTelemetryEnabled) {
        Serial1.print(F("auto-arm balancePoint="));
        Serial1.println(activeBalancePointDegrees);
      }
```

In `updateBalancePointLearning`, after the existing USB debug prints, add:

```cpp
    if (Config::EnableBluetoothTestControl && bluetoothTelemetryEnabled) {
      Serial1.print(F("balance-point saved="));
      Serial1.print(activeBalancePointDegrees);
      Serial1.print(F(" writes="));
      Serial1.println(balancePointStore.writeCounter());
    }
```

- [ ] **Step 11: Compile the sketch**

Run:

```powershell
arduino-cli compile --fqbn arduino:avr:mega SelfBalanceRobot
```

Expected: PASS. If the compiler reports a missing `Stream` print overload, keep the helper signatures as `Stream&`; Arduino `Stream` derives from `Print` and supports the used methods on AVR.

- [ ] **Step 12: Run native tests**

Run:

```powershell
make all
```

Expected: all native tests pass, including `test_command_parser`, `test_command_reader`, and `test_balance_point_store`.

- [ ] **Step 13: Commit sketch integration**

Run:

```powershell
git add SelfBalanceRobot/SelfBalanceRobot.ino SelfBalanceRobot/config.h
git commit -m "feat: wire bluetooth test command channel"
```

---

### Task 5: Document Bluetooth Cable-Free Testing

**Files:**
- Modify: `docs/bring-up.md`

- [ ] **Step 1: Update command documentation**

In `docs/bring-up.md`, rename `## Serial Commands` to:

```markdown
## USB and Bluetooth Commands
```

Replace the introductory sentence with:

```markdown
Commands are case-insensitive and newline-terminated. USB Serial Monitor uses `Serial` at 115200 baud. Bluetooth test control uses `Serial1` at 115200 baud when `EnableBluetoothTestControl` is true.
```

Extend the command table with:

```markdown
| `BP?` | Print current balance point, stored/default status, and EEPROM write count. |
| `BP SET <degrees>` | Persist an absolute balance point for auto-arm tests. Example: `bp set 0.85` |
| `BP <degrees>` | Short alias for `BP SET <degrees>`. |
| `BP CLEAR` | Clear learned EEPROM balance point, fall back to default, stop motors, and suppress auto-arm briefly. |
| `AUTO ON` / `AUTO OFF` | Enable or disable auto-arm until reset. |
| `LEARN ON` / `LEARN OFF` | Enable or disable balance-point EEPROM learning until reset. |
| `STATUS` | Print one diagnostic snapshot. |
| `TELEM ON` / `TELEM OFF` | Enable or disable periodic telemetry over Bluetooth. |
```

- [ ] **Step 2: Add a cable-free Bluetooth test section**

Add this section after `Cable-Free Auto-Arm`:

```markdown
## Bluetooth Cable-Free Test Flow

1. Upload over USB, then disconnect the USB cable.
2. Connect to the Bluetooth serial module at 115200 baud.
3. Send `STOP`, `AUTO OFF`, `LEARN OFF`, and `BP CLEAR`.
4. Send `STATUS` and confirm `angle=` changes when tipping forward/backward.
5. For manual tests, hold the robot still and send `ARM`.
6. If it falls forward immediately, stop, adjust the target in small steps, and retry. Use `TRIM <degrees>` for manual calibration sessions and `BP SET <degrees>` for auto-arm sessions.
7. Once a cable-free balance point works for short tests, send `BP SET <degrees>`, then test `AUTO ON`.
8. Turn `LEARN ON` back on only after the robot can balance without immediate divergence.
```

- [ ] **Step 3: Verify docs mention the manual setter**

Run:

```powershell
rg -n "BP SET|Bluetooth Cable-Free|TELEM|LEARN OFF" docs/bring-up.md docs/superpowers/specs/2026-05-29-bluetooth-test-control-channel-design.md
```

Expected: output includes both the spec and bring-up guide.

- [ ] **Step 4: Commit docs**

Run:

```powershell
git add docs/bring-up.md
git commit -m "docs: add bluetooth balance test flow"
```

---

### Task 6: Final Verification

**Files:**
- No source edits unless verification exposes a specific defect.

- [ ] **Step 1: Run all native tests**

Run:

```powershell
make all
```

Expected: all tests pass:

```text
test_balance_controller PASS
test_robot_state PASS
test_sensors PASS
test_auto_arm_controller PASS
test_balance_point_store PASS
test_balance_point_learner PASS
test_command_parser PASS
test_command_reader PASS
```

- [ ] **Step 2: Compile Arduino sketch**

Run:

```powershell
arduino-cli compile --fqbn arduino:avr:mega SelfBalanceRobot
```

Expected: compile succeeds for `arduino:avr:mega`.

- [ ] **Step 3: Inspect git status**

Run:

```powershell
git status --short --branch
```

Expected: branch is ahead of `origin/main`; only pre-existing untracked `ruvector.db` may remain.

- [ ] **Step 4: Request code review**

Use `superpowers:requesting-code-review` before claiming the feature complete. Ask the reviewer to focus on command parser correctness, STOP priority, EEPROM clear behavior, and sketch integration risk.

---

## Self-Review

Spec coverage:

- Shared USB/Bluetooth parser: Tasks 1, 2, and 4.
- Bluetooth on `Serial1` at 115200: Task 4.
- Existing commands preserved: Task 1 parser tests and Task 4 application helpers.
- `BP?`, `BP SET`, `BP CLEAR`, `AUTO`, `LEARN`, `STATUS`, and `TELEM`: Tasks 1 and 4.
- EEPROM bad-point recovery: Task 3 and Task 4.
- Cable-free test sequence: Task 5.
- Native tests and Arduino compile: Tasks 1, 2, 3, 4, and 6.

Placeholder scan:

- No placeholder sections are left.
- Every task has concrete files, commands, expected outputs, and code snippets.

Type consistency:

- `ParsedCommandAction`, `ParsedCommand`, `CommandParser`, and `CommandReader` names are consistent across tests, production code, and sketch integration.
- `BalancePointStore::clearBalancePoint(float fallbackDegrees)` is consistent in the test, header, implementation, and sketch integration.
