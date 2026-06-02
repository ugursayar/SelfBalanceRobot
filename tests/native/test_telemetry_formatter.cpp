#include <cassert>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

#include "../../SelfBalanceRobot/TelemetryFormatter.h"

class CaptureOutput {
public:
  void print(const char* text) { buffer_ << text; }
  void print(char text) { buffer_ << text; }
  void print(int value) { buffer_ << value; }
  void print(uint32_t value) { buffer_ << value; }
  void print(unsigned long value) { buffer_ << value; }
  void print(float value) { buffer_ << value; }
  void println(const char* text) { buffer_ << text << '\n'; }
  void println(float value) { buffer_ << value << '\n'; }
  void println(int value) { buffer_ << value << '\n'; }
  void println(unsigned long value) { buffer_ << value << '\n'; }
  void println() { buffer_ << '\n'; }
  std::string str() const { return buffer_.str(); }

private:
  std::ostringstream buffer_;
};

static TelemetrySnapshot sample() {
  TelemetrySnapshot snapshot;
  snapshot.mode = RobotMode::Balancing;
  snapshot.loopMicros = 10000;
  snapshot.angleDegrees = -1.25f;
  snapshot.uprightAngleDegrees = 0.0f;
  snapshot.targetAngleDegrees = 0.75f;
  snapshot.trimDegrees = -2.3f;
  snapshot.activeBalancePointDegrees = 0.75f;
  snapshot.storedBalancePoint = true;
  snapshot.autoArmEnabled = true;
  snapshot.autoAngleErrorDegrees = 0.1f;
  snapshot.gyroFresh = true;
  snapshot.gyroRateDegPerSec = 2.0f;
  snapshot.filteredRateDegPerSec = 1.5f;
  snapshot.rawBalanceOutput = -10;
  snapshot.balanceOutput = -16;
  snapshot.averageSpeedRpm = 0.0f;
  snapshot.averagePositionDegrees = 5.0f;
  snapshot.travelHoldTargetCorrectionDegrees = 0.01f;
  snapshot.leftMotor = -16;
  snapshot.rightMotor = -16;
  snapshot.leftPwm = -16;
  snapshot.rightPwm = -16;
  snapshot.kp = 42.0f;
  snapshot.ki = 0.0f;
  snapshot.kd = 1.0f;
  snapshot.runtime.balanceTicks = 12;
  snapshot.runtime.lastWorkMicros = 3000;
  snapshot.runtime.maxWorkMicros = 4200;
  return snapshot;
}

static void test_status_contains_runtime_and_control_fields() {
  CaptureOutput out;

  TelemetryFormatter::printStatus(out, sample());
  const std::string text = out.str();

  assert(text.find("status mode=balancing") != std::string::npos);
  assert(text.find(" angle=") != std::string::npos);
  assert(text.find(" target=") != std::string::npos);
  assert(text.find(" raw=") != std::string::npos);
  assert(text.find(" balance=") != std::string::npos);
  assert(text.find(" loopTicks=") != std::string::npos);
  assert(text.find(" workUs=") != std::string::npos);
}

static void test_debug_uses_shorter_field_set_than_status() {
  CaptureOutput out;

  TelemetryFormatter::printDebug(out, sample());
  const std::string text = out.str();

  assert(text.find("mode=balancing") != std::string::npos);
  assert(text.find("loopUs=") != std::string::npos);
  assert(text.find("resetRaw=") == std::string::npos);
  assert(text.find("feedbackFull=") == std::string::npos);
}

static void test_bluetooth_telemetry_keeps_telem_prefix() {
  CaptureOutput out;

  TelemetryFormatter::printBluetoothTelemetry(out, sample());
  const std::string text = out.str();

  assert(text.find("telem mode=balancing") == 0);
  assert(text.find(" auto=") != std::string::npos);
  assert(text.find(" gyroFresh=") != std::string::npos);
}

int main() {
  test_status_contains_runtime_and_control_fields();
  test_debug_uses_shorter_field_set_than_status();
  test_bluetooth_telemetry_keeps_telem_prefix();

  std::cout << "test_telemetry_formatter PASS\n";
  return EXIT_SUCCESS;
}
