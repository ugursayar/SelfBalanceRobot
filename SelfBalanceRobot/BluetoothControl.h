#ifndef BLUETOOTH_CONTROL_H
#define BLUETOOTH_CONTROL_H

#include "RobotTypes.h"
#include "config.h"

class BluetoothControl {
public:
  explicit BluetoothControl(Stream* stream = 0);

  void begin(Stream& stream);
  void attach(Stream* stream);
  const ControlCommand& update(unsigned long nowMillis);
  const ControlCommand& current() const;
  void consumeTuning();
  void consumeTrim();

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
  bool commandEquals(const char* actual, const char* expected) const;
  bool parseIntegerToken(const char* token, long* value) const;
  bool parseFloatToken(const char* token, float* value) const;
  bool tuningInRange(float kp, float ki, float kd) const;
  bool trimInRange(float trimDegrees) const;
  int16_t clampCommand(long value, int16_t limit) const;
};

#endif
