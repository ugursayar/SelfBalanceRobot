#ifndef BLUETOOTH_SERIAL_PORT_H
#define BLUETOOTH_SERIAL_PORT_H

#include <stdint.h>

// Hardware UART map:
//   Serial  - USB debug monitor and testing fallback
//   Serial2 - RPi primary control link, reserved for future use
//   Serial3 - Makeblock Bluetooth module
#define ROBOT_BLUETOOTH_SERIAL Serial3

namespace RobotBluetoothSerial {
constexpr uint8_t ReservedRpiHardwareSerialIndex = 2;
constexpr uint8_t HardwareSerialIndex = 3;
constexpr const char* Name = "Serial3";
}

#endif
