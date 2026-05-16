#ifndef BLUETOOTH_CONTROL_H
#define BLUETOOTH_CONTROL_H

#include "RobotTypes.h"
#include "config.h"

class BluetoothControl {
public:
  explicit BluetoothControl(Stream* stream = 0);

  void attach(Stream* stream);
  const ControlCommand& update(unsigned long nowMillis);
  const ControlCommand& current() const;

private:
  static const uint8_t BufferSize = 48;

  Stream* stream_;
  ControlCommand command_;
  char buffer_[BufferSize];
  uint8_t length_;
  bool overflow_;

  void readByte(char value, unsigned long nowMillis);
  void parseLine(char* line, unsigned long nowMillis);
  void markReceived(unsigned long nowMillis);
  bool parseIntegerToken(const char* token, long* value) const;
  int16_t clampCommand(long value, int16_t limit) const;
};

#endif
