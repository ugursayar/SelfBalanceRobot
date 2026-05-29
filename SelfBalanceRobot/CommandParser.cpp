#include "CommandParser.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static bool tokenEquals(const char* token, const char* expected) {
  return token != nullptr && strcmp(token, expected) == 0;
}

ParsedCommand CommandParser::parse(char* line) const {
  ParsedCommand command;

  if (line == nullptr) {
    return invalid();
  }

  normalize(line);

  if (line[0] == '\0') {
    return command;
  }

  if (strcmp(line, "arm") == 0) {
    command.action = ParsedCommandAction::Arm;
    return command;
  }
  if (strcmp(line, "stop") == 0) {
    command.action = ParsedCommandAction::Stop;
    return command;
  }
  if (strcmp(line, "m+") == 0) {
    command.action = ParsedCommandAction::MotorBackward;
    return command;
  }
  if (strcmp(line, "m-") == 0) {
    command.action = ParsedCommandAction::MotorForward;
    return command;
  }
  if (strcmp(line, "status") == 0) {
    command.action = ParsedCommandAction::Status;
    return command;
  }
  if (strcmp(line, "bp?") == 0) {
    command.action = ParsedCommandAction::BalancePointQuery;
    return command;
  }

  char* firstToken = strtok(line, " ");
  if (firstToken == nullptr) {
    return command;
  }

  if (tokenEquals(firstToken, "pid")) {
    char* kp = strtok(nullptr, " ");
    char* ki = strtok(nullptr, " ");
    char* kd = strtok(nullptr, " ");
    if (kp == nullptr || ki == nullptr || kd == nullptr || strtok(nullptr, " ") != nullptr) {
      return invalid();
    }
    if (!parseFloatToken(kp, command.first) || !parseFloatToken(ki, command.second) ||
        !parseFloatToken(kd, command.third)) {
      return invalid();
    }
    command.action = ParsedCommandAction::SetPid;
    return command;
  }

  if (tokenEquals(firstToken, "trim")) {
    char* trim = strtok(nullptr, " ");
    if (trim == nullptr || strtok(nullptr, " ") != nullptr || !parseFloatToken(trim, command.first)) {
      return invalid();
    }
    command.action = ParsedCommandAction::SetTrim;
    return command;
  }

  if (tokenEquals(firstToken, "bp")) {
    char* secondToken = strtok(nullptr, " ");
    if (secondToken == nullptr) {
      return invalid();
    }

    if (tokenEquals(secondToken, "clear")) {
      if (strtok(nullptr, " ") != nullptr) {
        return invalid();
      }
      command.action = ParsedCommandAction::BalancePointClear;
      return command;
    }

    if (tokenEquals(secondToken, "set")) {
      char* degrees = strtok(nullptr, " ");
      if (degrees == nullptr || strtok(nullptr, " ") != nullptr ||
          !parseFloatToken(degrees, command.first)) {
        return invalid();
      }
      command.action = ParsedCommandAction::BalancePointSet;
      return command;
    }

    if (strtok(nullptr, " ") != nullptr || !parseFloatToken(secondToken, command.first)) {
      return invalid();
    }
    command.action = ParsedCommandAction::BalancePointSet;
    return command;
  }

  if (tokenEquals(firstToken, "auto") || tokenEquals(firstToken, "learn") ||
      tokenEquals(firstToken, "telem")) {
    char* state = strtok(nullptr, " ");
    if (state == nullptr || strtok(nullptr, " ") != nullptr) {
      return invalid();
    }

    if (tokenEquals(firstToken, "auto") && tokenEquals(state, "on")) {
      command.action = ParsedCommandAction::AutoOn;
      return command;
    }
    if (tokenEquals(firstToken, "auto") && tokenEquals(state, "off")) {
      command.action = ParsedCommandAction::AutoOff;
      return command;
    }
    if (tokenEquals(firstToken, "learn") && tokenEquals(state, "on")) {
      command.action = ParsedCommandAction::LearnOn;
      return command;
    }
    if (tokenEquals(firstToken, "learn") && tokenEquals(state, "off")) {
      command.action = ParsedCommandAction::LearnOff;
      return command;
    }
    if (tokenEquals(firstToken, "telem") && tokenEquals(state, "on")) {
      command.action = ParsedCommandAction::TelemetryOn;
      return command;
    }
    if (tokenEquals(firstToken, "telem") && tokenEquals(state, "off")) {
      command.action = ParsedCommandAction::TelemetryOff;
      return command;
    }
    return invalid();
  }

  return invalid();
}

void CommandParser::normalize(char* line) const {
  char* read = line;
  char* write = line;
  bool inSpace = true;

  while (*read != '\0') {
    char c = *read++;
    if (c >= 'A' && c <= 'Z') {
      c = static_cast<char>(c - 'A' + 'a');
    }
    if (c == '\t') {
      c = ' ';
    }

    if (c == ' ') {
      if (!inSpace) {
        *write++ = c;
        inSpace = true;
      }
    } else {
      *write++ = c;
      inSpace = false;
    }
  }

  if (write > line && *(write - 1) == ' ') {
    --write;
  }
  *write = '\0';
}

bool CommandParser::parseFloatToken(const char* text, float& value) const {
  char* end = nullptr;
  double parsed = strtod(text, &end);
  if (end == text || *end != '\0' || !isfinite(parsed)) {
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
