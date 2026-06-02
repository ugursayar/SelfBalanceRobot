#include <cassert>
#include <cstdlib>
#include <iostream>

#define Serial1 1
#define Serial2 2
#define Serial3 3
#include "../../SelfBalanceRobot/BluetoothSerialPort.h"

static void test_bluetooth_uses_serial3_and_leaves_serial2_for_rpi() {
  static_assert(RobotBluetoothSerial::ReservedRpiHardwareSerialIndex == 2,
                "Serial2 is reserved for the RPi primary control link");
  static_assert(RobotBluetoothSerial::HardwareSerialIndex == 3,
                "Bluetooth module is wired to Serial3");
  assert(ROBOT_BLUETOOTH_SERIAL == 3);
}

int main() {
  test_bluetooth_uses_serial3_and_leaves_serial2_for_rpi();

  std::cout << "test_bluetooth_serial_port PASS\n";
  return EXIT_SUCCESS;
}
