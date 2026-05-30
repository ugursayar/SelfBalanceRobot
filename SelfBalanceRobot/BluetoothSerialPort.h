#ifndef BLUETOOTH_SERIAL_PORT_H
#define BLUETOOTH_SERIAL_PORT_H

#include <stdint.h>

// Makeblock MegaPi examples listen on both hardware UARTs used by the RJ25
// shield and wireless module variants.
#define ROBOT_BLUETOOTH_SERIAL_PRIMARY Serial2
#define ROBOT_BLUETOOTH_SERIAL_SECONDARY Serial3
#define ROBOT_BLUETOOTH_SERIAL ROBOT_BLUETOOTH_SERIAL_PRIMARY

namespace RobotBluetoothSerial {
constexpr uint8_t PrimaryHardwareSerialIndex = 2;
constexpr uint8_t SecondaryHardwareSerialIndex = 3;
constexpr uint8_t ProbePortCount = 2;
constexpr const char* Name = "Serial2+Serial3";
}

#endif
