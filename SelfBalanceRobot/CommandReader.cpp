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
