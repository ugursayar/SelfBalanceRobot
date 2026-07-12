#include <cassert>
#include <cstdlib>
#include <iostream>

#include "../../SelfBalanceRobot/CommandReader.h"

class FakeStream : public Stream {
public:
  explicit FakeStream(const char* text) : index_(0), length_(0) {
    append(text);
  }

  void append(const char* text) {
    while (*text != '\0' && length_ < kCapacity - 1) {
      text_[length_++] = *text++;
    }
    text_[length_] = '\0';
  }

  int available() {
    return index_ >= length_ ? 0 : 1;
  }

  int read() {
    if (index_ >= length_) {
      return -1;
    }
    return text_[index_++];
  }

private:
  static const size_t kCapacity = 128;

  char text_[kCapacity];
  size_t index_;
  size_t length_;
};

static void test_reader_waits_for_newline() {
  FakeStream stream("arm");
  CommandReader reader;
  reader.begin(stream);

  ParsedCommand command;
  assert(!reader.readCommand(command, 10));
}

static void test_reader_accepts_stop_without_newline() {
  FakeStream stream("STOP");
  CommandReader reader;
  reader.begin(stream);

  ParsedCommand command;
  assert(reader.readCommand(command, 11));
  assert(command.action == ParsedCommandAction::Stop);
  assert(command.receivedMillis == 11);
}

static void test_reader_accepts_status_without_newline() {
  FakeStream stream("status");
  CommandReader reader;
  reader.begin(stream);

  ParsedCommand command;
  assert(reader.readCommand(command, 12));
  assert(command.action == ParsedCommandAction::Status);
  assert(command.receivedMillis == 12);
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

static void test_reader_ignores_empty_newline_before_command() {
  FakeStream stream("\narm\n");
  CommandReader reader;
  reader.begin(stream);

  ParsedCommand command;
  assert(reader.readCommand(command, 10));
  assert(command.action == ParsedCommandAction::Arm);
  assert(command.receivedMillis == 10);
}

static void test_reader_handles_crlf_and_following_command() {
  FakeStream stream("arm\r\nstop\n");
  CommandReader reader;
  reader.begin(stream);

  ParsedCommand first;
  ParsedCommand second;
  assert(reader.readCommand(first, 11));
  assert(first.action == ParsedCommandAction::Arm);
  assert(first.receivedMillis == 11);
  assert(reader.readCommand(second, 12));
  assert(second.action == ParsedCommandAction::Stop);
  assert(second.receivedMillis == 12);
}

static void test_reader_returns_invalid_for_overflowed_line() {
  FakeStream stream(
      "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\n");
  CommandReader reader;
  reader.begin(stream);

  ParsedCommand command;
  assert(reader.readCommand(command, 20));
  assert(command.action == ParsedCommandAction::Invalid);
  assert(command.receivedMillis == 20);
}

static void test_reader_recovers_after_overflow_terminator() {
  FakeStream stream(
      "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\nstop\n");
  CommandReader reader;
  reader.begin(stream);

  ParsedCommand overflowed;
  ParsedCommand valid;
  assert(reader.readCommand(overflowed, 21));
  assert(overflowed.action == ParsedCommandAction::Invalid);
  assert(reader.readCommand(valid, 22));
  assert(valid.action == ParsedCommandAction::Stop);
  assert(valid.receivedMillis == 22);
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

  usb.append("m\n");
  assert(usbReader.readCommand(command, 3));
  assert(command.action == ParsedCommandAction::Arm);
  assert(command.receivedMillis == 3);
}

static void test_reset_discards_partial_line() {
  FakeStream stream("ar");
  CommandReader reader;
  reader.begin(stream);

  ParsedCommand command;
  assert(!reader.readCommand(command, 1));

  reader.reset();
  stream.append("m\nstop\n");

  assert(reader.readCommand(command, 2));
  assert(command.action == ParsedCommandAction::Invalid);
  assert(command.receivedMillis == 2);
  assert(reader.readCommand(command, 3));
  assert(command.action == ParsedCommandAction::Stop);
  assert(command.receivedMillis == 3);
}

static void test_reset_discards_overflow_state() {
  FakeStream stream("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
  CommandReader reader;
  reader.begin(stream);

  ParsedCommand command;
  assert(!reader.readCommand(command, 4));

  reader.reset();
  stream.append("stop\n");

  assert(reader.readCommand(command, 5));
  assert(command.action == ParsedCommandAction::Stop);
  assert(command.receivedMillis == 5);
}

int main() {
  test_reader_waits_for_newline();
  test_reader_accepts_stop_without_newline();
  test_reader_accepts_status_without_newline();
  test_reader_parses_complete_line_with_timestamp();
  test_reader_returns_one_command_per_call();
  test_reader_ignores_empty_newline_before_command();
  test_reader_handles_crlf_and_following_command();
  test_reader_returns_invalid_for_overflowed_line();
  test_reader_recovers_after_overflow_terminator();
  test_readers_keep_independent_buffers();
  test_reset_discards_partial_line();
  test_reset_discards_overflow_state();

  std::cout << "test_command_reader PASS\n";
  return EXIT_SUCCESS;
}
