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
