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
