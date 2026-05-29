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
