#include "BluetoothControl.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace {
char toUpperAscii(char value) {
  if (value >= 'a' && value <= 'z') {
    return static_cast<char>(value - ('a' - 'A'));
  }
  return value;
}
} // namespace

BluetoothControl::BluetoothControl(Stream* stream)
    : stream_(stream), command_(), buffer_(), length_(0), overflow_(false) {}

void BluetoothControl::begin(Stream& stream) { attach(&stream); }

void BluetoothControl::attach(Stream* stream) {
  stream_ = stream;
  length_ = 0;
  overflow_ = false;
}

const ControlCommand& BluetoothControl::update(unsigned long nowMillis) {
  if (!stream_) {
    return command_;
  }

  while (stream_->available() > 0) {
    readByte(static_cast<char>(stream_->read()), nowMillis);
  }

  return command_;
}

const ControlCommand& BluetoothControl::current() const { return command_; }

void BluetoothControl::consumeTuning() { command_.hasTuning = false; }

void BluetoothControl::consumeTrim() { command_.hasTrim = false; }

void BluetoothControl::readByte(char value, unsigned long nowMillis) {
  if (value == '\r') {
    return;
  }

  if (value == '\n') {
    if (!overflow_) {
      buffer_[length_] = '\0';
      parseLine(buffer_, nowMillis);
    }
    length_ = 0;
    overflow_ = false;
    return;
  }

  if (overflow_) {
    return;
  }

  if (length_ >= BufferSize - 1) {
    length_ = 0;
    overflow_ = true;
    return;
  }

  buffer_[length_++] = value;
}

void BluetoothControl::parseLine(char* line, unsigned long nowMillis) {
  char* command = strtok(line, " \t");
  if (!command) {
    return;
  }

  if (commandEquals(command, "ARM")) {
    command_.arm = true;
    command_.stop = false;
    markReceived(nowMillis);
    return;
  }

  if (commandEquals(command, "STOP")) {
    command_.arm = false;
    command_.stop = true;
    command_.driveEnabled = false;
    command_.forward = 0;
    command_.turn = 0;
    markReceived(nowMillis);
    return;
  }

  if (commandEquals(command, "BALANCE")) {
    command_.arm = false;
    command_.stop = false;
    command_.driveEnabled = false;
    command_.forward = 0;
    command_.turn = 0;
    markReceived(nowMillis);
    return;
  }

  if (commandEquals(command, "DRIVE")) {
    char* forward = strtok(0, " \t");
    char* turn = strtok(0, " \t");
    if (!forward || !turn) {
      return;
    }

    long forwardValue = 0;
    long turnValue = 0;
    if (!parseIntegerToken(forward, &forwardValue) ||
        !parseIntegerToken(turn, &turnValue)) {
      return;
    }

    command_.arm = false;
    command_.stop = false;
    command_.driveEnabled = true;
    command_.forward = clampCommand(forwardValue, Config::MaxDriveCommand);
    command_.turn = clampCommand(turnValue, Config::MaxTurnCommand);
    markReceived(nowMillis);
    return;
  }

  if (commandEquals(command, "PID") && Config::EnableRuntimeTuning) {
    char* kp = strtok(0, " \t");
    char* ki = strtok(0, " \t");
    char* kd = strtok(0, " \t");
    if (!kp || !ki || !kd) {
      return;
    }

    float parsedKp = 0.0f;
    float parsedKi = 0.0f;
    float parsedKd = 0.0f;
    if (!parseFloatToken(kp, &parsedKp) ||
        !parseFloatToken(ki, &parsedKi) ||
        !parseFloatToken(kd, &parsedKd) ||
        !tuningInRange(parsedKp, parsedKi, parsedKd)) {
      return;
    }

    command_.arm = false;
    command_.stop = false;
    command_.tuneKp = parsedKp;
    command_.tuneKi = parsedKi;
    command_.tuneKd = parsedKd;
    command_.hasTuning = true;
    markReceived(nowMillis);
    return;
  }

  if (commandEquals(command, "TRIM") && Config::EnableRuntimeTuning) {
    char* trim = strtok(0, " \t");
    if (!trim) {
      return;
    }

    float parsedTrim = 0.0f;
    if (!parseFloatToken(trim, &parsedTrim) || !trimInRange(parsedTrim)) {
      return;
    }

    command_.arm = false;
    command_.stop = false;
    command_.tuneTrimDegrees = parsedTrim;
    command_.hasTrim = true;
    markReceived(nowMillis);
  }
}

void BluetoothControl::markReceived(unsigned long nowMillis) {
  command_.receivedMillis = nowMillis;
}

bool BluetoothControl::commandEquals(const char* actual,
                                     const char* expected) const {
  if (!actual || !expected) {
    return false;
  }

  while (*actual != '\0' && *expected != '\0') {
    if (toUpperAscii(*actual) != *expected) {
      return false;
    }
    ++actual;
    ++expected;
  }

  return *actual == '\0' && *expected == '\0';
}

bool BluetoothControl::parseIntegerToken(const char* token, long* value) const {
  if (!token || !*token || !value) {
    return false;
  }

  char* end = 0;
  const long parsed = strtol(token, &end, 10);
  if (end == token || *end != '\0') {
    return false;
  }

  *value = parsed;
  return true;
}

bool BluetoothControl::parseFloatToken(const char* token, float* value) const {
  if (!token || !*token || !value) {
    return false;
  }

  char* end = 0;
  const double parsed = strtod(token, &end);
  if (end == token || *end != '\0' || !isfinite(parsed)) {
    return false;
  }

  *value = static_cast<float>(parsed);
  return isfinite(*value);
}

bool BluetoothControl::tuningInRange(float kp, float ki, float kd) const {
  return kp >= Config::MinRuntimeKp && kp <= Config::MaxRuntimeKp &&
         ki >= Config::MinRuntimeKi && ki <= Config::MaxRuntimeKi &&
         kd >= Config::MinRuntimeKd && kd <= Config::MaxRuntimeKd;
}

bool BluetoothControl::trimInRange(float trimDegrees) const {
  return trimDegrees >= Config::MinRuntimeTrimDegrees &&
         trimDegrees <= Config::MaxRuntimeTrimDegrees;
}

int16_t BluetoothControl::clampCommand(long value, int16_t limit) const {
  const long positiveLimit = limit < 0 ? -static_cast<long>(limit) : limit;
  if (value > positiveLimit) {
    return static_cast<int16_t>(positiveLimit);
  }
  if (value < -positiveLimit) {
    return static_cast<int16_t>(-positiveLimit);
  }
  return static_cast<int16_t>(value);
}
