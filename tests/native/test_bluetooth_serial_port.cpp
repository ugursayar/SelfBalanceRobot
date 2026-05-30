#include <cassert>
#include <cstdlib>
#include <iostream>

#define Serial1 1
#define Serial2 2
#define Serial3 3
#include "../../SelfBalanceRobot/BluetoothSerialPort.h"

static void test_megapi_bluetooth_probes_serial2_and_serial3() {
  static_assert(RobotBluetoothSerial::PrimaryHardwareSerialIndex == 2,
                "MegaPi RJ25 port 5 maps to Serial2 on ATmega2560");
  static_assert(RobotBluetoothSerial::SecondaryHardwareSerialIndex == 3,
                "Makeblock MegaPi firmware also accepts Bluetooth on Serial3");
  static_assert(RobotBluetoothSerial::ProbePortCount == 2,
                "Bluetooth test control should probe both MegaPi candidates");
  assert(ROBOT_BLUETOOTH_SERIAL_PRIMARY == 2);
  assert(ROBOT_BLUETOOTH_SERIAL_SECONDARY == 3);
}

int main() {
  test_megapi_bluetooth_probes_serial2_and_serial3();

  std::cout << "test_bluetooth_serial_port PASS\n";
  return EXIT_SUCCESS;
}
