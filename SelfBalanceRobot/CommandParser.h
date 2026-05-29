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
