#include "CommandReader.h"

namespace {
char lowerAscii(char value) {
  if (value >= 'A' && value <= 'Z') {
    return static_cast<char>(value - 'A' + 'a');
  }
  return value;
}

bool bufferEqualsToken(const char* buffer, uint8_t length,
                       const char* token) {
  uint8_t index = 0;
  while (index < length && token[index] != '\0') {
    if (lowerAscii(buffer[index]) != token[index]) {
      return false;
    }
    ++index;
  }
  return index == length && token[index] == '\0';
}

bool parseImmediateCommand(const char* buffer, uint8_t length,
                           ParsedCommand& command) {
  if (bufferEqualsToken(buffer, length, "stop")) {
    command = ParsedCommand();
    command.action = ParsedCommandAction::Stop;
    return true;
  }

  if (bufferEqualsToken(buffer, length, "status")) {
    command = ParsedCommand();
    command.action = ParsedCommandAction::Status;
    return true;
  }

  return false;
}
}

CommandReader::CommandReader()
    : stream_(nullptr), length_(0), overflow_(false) {
  buffer_[0] = '\0';
}

void CommandReader::begin(Stream& stream) {
  stream_ = &stream;
  reset();
}

void CommandReader::reset() {
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
      buffer_[length_] = '\0';
      if (!overflow_ && stream_->available() == 0 &&
          parseImmediateCommand(buffer_, length_, command)) {
        command.receivedMillis = nowMillis;
        length_ = 0;
        buffer_[0] = '\0';
        return true;
      }
    } else {
      overflow_ = true;
    }
  }

  return false;
}
