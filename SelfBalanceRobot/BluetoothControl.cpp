#include "BluetoothControl.h"

#include <stdlib.h>
#include <string.h>

BluetoothControl::BluetoothControl(Stream* stream)
    : stream_(stream), command_(), buffer_(), length_(0), overflow_(false) {}

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

  if (strcmp(command, "ARM") == 0) {
    command_.arm = true;
    command_.stop = false;
    markReceived(nowMillis);
    return;
  }

  if (strcmp(command, "STOP") == 0) {
    command_.stop = true;
    command_.driveEnabled = false;
    command_.forward = 0;
    command_.turn = 0;
    markReceived(nowMillis);
    return;
  }

  if (strcmp(command, "BALANCE") == 0) {
    command_.stop = false;
    command_.driveEnabled = false;
    command_.forward = 0;
    command_.turn = 0;
    markReceived(nowMillis);
    return;
  }

  if (strcmp(command, "DRIVE") == 0) {
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

    command_.stop = false;
    command_.driveEnabled = true;
    command_.forward = clampCommand(forwardValue, Config::MaxDriveCommand);
    command_.turn = clampCommand(turnValue, Config::MaxTurnCommand);
    markReceived(nowMillis);
    return;
  }

  if (strcmp(command, "PID") == 0 && Config::EnableRuntimeTuning) {
    char* kp = strtok(0, " \t");
    char* ki = strtok(0, " \t");
    char* kd = strtok(0, " \t");
    if (!kp || !ki || !kd) {
      return;
    }

    command_.tuneKp = static_cast<float>(atof(kp));
    command_.tuneKi = static_cast<float>(atof(ki));
    command_.tuneKd = static_cast<float>(atof(kd));
    command_.hasTuning = true;
    markReceived(nowMillis);
  }
}

void BluetoothControl::markReceived(unsigned long nowMillis) {
  command_.receivedMillis = nowMillis;
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
