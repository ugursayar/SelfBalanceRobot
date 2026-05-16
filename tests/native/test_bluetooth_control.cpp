#include <cassert>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#define private public
#include "../../SelfBalanceRobot/BluetoothControl.h"
#undef private

class FakeStream : public Stream {
public:
  explicit FakeStream(const char* data) : data_(data), position_(0) {}

  int available() override {
    return position_ < data_.size() ? static_cast<int>(data_.size() - position_)
                                    : 0;
  }

  int read() override {
    if (position_ >= data_.size()) {
      return -1;
    }
    return data_[position_++];
  }

private:
  std::string data_;
  std::size_t position_;
};

static void test_begin_attaches_stream_and_resets_buffer_state() {
  FakeStream stream("DRIVE 12 -7\n");
  BluetoothControl bluetooth;
  bluetooth.length_ = 8;
  bluetooth.overflow_ = true;

  bluetooth.begin(stream);
  const ControlCommand& command = bluetooth.update(42);

  assert(bluetooth.stream_ == &stream);
  assert(bluetooth.length_ == 0);
  assert(!bluetooth.overflow_);
  assert(command.driveEnabled);
  assert(command.forward == 12);
  assert(command.turn == -7);
  assert(command.receivedMillis == 42);
}

static ControlCommand parseCommand(const char* data, unsigned long nowMillis) {
  FakeStream stream(data);
  BluetoothControl bluetooth;
  bluetooth.begin(stream);
  return bluetooth.update(nowMillis);
}

static void test_arm_command_sets_arm_and_clears_stop() {
  ControlCommand command = parseCommand("ARM\n", 10);

  assert(command.arm);
  assert(!command.stop);
  assert(command.receivedMillis == 10);
}

static void test_stop_clears_drive_outputs() {
  FakeStream stream("DRIVE 20 -5\nSTOP\n");
  BluetoothControl bluetooth;
  bluetooth.begin(stream);

  const ControlCommand& command = bluetooth.update(25);

  assert(command.stop);
  assert(!command.driveEnabled);
  assert(command.forward == 0);
  assert(command.turn == 0);
  assert(command.receivedMillis == 25);
}

static void test_non_arm_commands_clear_sticky_arm_intent() {
  FakeStream balanceStream("ARM\nBALANCE\n");
  BluetoothControl balanceBluetooth;
  balanceBluetooth.begin(balanceStream);
  ControlCommand command = balanceBluetooth.update(26);
  assert(!command.arm);
  assert(!command.stop);
  assert(!command.driveEnabled);

  FakeStream driveStream("ARM\nDRIVE 3 4\n");
  BluetoothControl driveBluetooth;
  driveBluetooth.begin(driveStream);
  command = driveBluetooth.update(27);
  assert(!command.arm);
  assert(!command.stop);
  assert(command.driveEnabled);
  assert(command.forward == 3);
  assert(command.turn == 4);

  FakeStream stopStream("ARM\nSTOP\n");
  BluetoothControl stopBluetooth;
  stopBluetooth.begin(stopStream);
  command = stopBluetooth.update(28);
  assert(!command.arm);
  assert(command.stop);

  FakeStream pidStream("ARM\nPID 12 0 1\n");
  BluetoothControl pidBluetooth;
  pidBluetooth.begin(pidStream);
  command = pidBluetooth.update(29);
  assert(!command.arm);
  assert(!command.stop);
  assert(command.hasTuning);
}

static void test_drive_clamps_to_config_limits() {
  ControlCommand command = parseCommand("DRIVE 500 -500\n", 30);

  assert(command.driveEnabled);
  assert(command.forward == Config::MaxDriveCommand);
  assert(command.turn == -Config::MaxTurnCommand);
}

static void test_malformed_drive_is_rejected_without_changing_previous_command() {
  FakeStream stream("DRIVE 10 4\nDRIVE nope 7\n");
  BluetoothControl bluetooth;
  bluetooth.begin(stream);

  const ControlCommand& command = bluetooth.update(40);

  assert(command.driveEnabled);
  assert(command.forward == 10);
  assert(command.turn == 4);
  assert(command.receivedMillis == 40);
}

static void test_crlf_line_endings_are_accepted() {
  ControlCommand command = parseCommand("DRIVE 12 -7\r\n", 50);

  assert(command.driveEnabled);
  assert(command.forward == 12);
  assert(command.turn == -7);
}

static void test_valid_pid_is_one_shot_after_consume() {
  FakeStream stream("PID 12.5 0.1 0.9\n");
  BluetoothControl bluetooth;
  bluetooth.begin(stream);

  const ControlCommand& command = bluetooth.update(60);

  assert(command.hasTuning);
  assert(std::fabs(command.tuneKp - 12.5f) < 0.001f);
  assert(std::fabs(command.tuneKi - 0.1f) < 0.001f);
  assert(std::fabs(command.tuneKd - 0.9f) < 0.001f);

  bluetooth.consumeTuning();

  assert(!bluetooth.current().hasTuning);
}

static void test_malformed_non_finite_and_extreme_pid_are_rejected() {
  FakeStream stream("PID no 0 1\nPID inf 0 1\nPID 9999 0 1\n");
  BluetoothControl bluetooth;
  bluetooth.begin(stream);

  const ControlCommand& command = bluetooth.update(70);

  assert(!command.hasTuning);
  assert(command.receivedMillis == 0);
}

int main() {
  test_begin_attaches_stream_and_resets_buffer_state();
  test_arm_command_sets_arm_and_clears_stop();
  test_stop_clears_drive_outputs();
  test_non_arm_commands_clear_sticky_arm_intent();
  test_drive_clamps_to_config_limits();
  test_malformed_drive_is_rejected_without_changing_previous_command();
  test_crlf_line_endings_are_accepted();
  test_valid_pid_is_one_shot_after_consume();
  test_malformed_non_finite_and_extreme_pid_are_rejected();

  std::cout << "test_bluetooth_control PASS\n";
  return EXIT_SUCCESS;
}
