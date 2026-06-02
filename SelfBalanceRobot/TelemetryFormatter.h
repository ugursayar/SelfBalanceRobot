#ifndef TELEMETRY_FORMATTER_H
#define TELEMETRY_FORMATTER_H

#include "ResetDiagnostics.h"
#include "RobotTypes.h"
#include "RuntimeStats.h"

struct TelemetrySnapshot {
  RobotMode mode = RobotMode::Disarmed;
  unsigned long loopMicros = 0;
  float angleDegrees = 0.0f;
  float uprightAngleDegrees = 0.0f;
  float targetAngleDegrees = 0.0f;
  float errorDegrees = 0.0f;
  float trimDegrees = 0.0f;
  float activeBalancePointDegrees = 0.0f;
  bool storedBalancePoint = false;
  uint8_t resetRaw = 0;
  bool autoArmEnabled = false;
  float autoAngleErrorDegrees = 0.0f;
  bool gyroFresh = false;
  float gyroRateDegPerSec = 0.0f;
  float filteredRateDegPerSec = 0.0f;
  int16_t rawBalanceOutput = 0;
  int16_t balanceOutput = 0;
  float averageSpeedRpm = 0.0f;
  float averagePositionDegrees = 0.0f;
  float travelHoldTargetCorrectionDegrees = 0.0f;
  int16_t leftMotor = 0;
  int16_t rightMotor = 0;
  int16_t leftPwm = 0;
  int16_t rightPwm = 0;
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  RuntimeStatsSnapshot runtime;
};

class TelemetryFormatter {
public:
  template <typename Out>
  static void printMode(Out& out, RobotMode mode) {
    switch (mode) {
    case RobotMode::Disarmed:
      out.print("disarmed");
      break;
    case RobotMode::Calibrating:
      out.print("calibrating");
      break;
    case RobotMode::Balancing:
      out.print("balancing");
      break;
    case RobotMode::Fault:
      out.print("fault");
      break;
    }
  }

  template <typename Out>
  static void printStatus(Out& out, const TelemetrySnapshot& snapshot) {
    out.print("status mode=");
    printMode(out, snapshot.mode);
    printCommonControlFields(out, snapshot);
    out.print(" stored=");
    out.print(snapshot.storedBalancePoint ? "yes" : "no");
    out.print(" reset=");
    printResetCause(out, snapshot.resetRaw);
    out.print(" resetRaw=0x");
    printHexByte(out, snapshot.resetRaw);
    out.print(" ki=");
    out.print(snapshot.ki);
    printRuntimeFields(out, snapshot.runtime);
    out.println();
  }

  template <typename Out>
  static void printDebug(Out& out, const TelemetrySnapshot& snapshot) {
    out.print("mode=");
    printMode(out, snapshot.mode);
    out.print(" loopUs=");
    out.print(snapshot.loopMicros);
    printCommonControlFields(out, snapshot);
    out.print(" err=");
    out.print(snapshot.errorDegrees);
    out.print(" lpwm=");
    out.print(snapshot.leftPwm);
    out.print(" rpwm=");
    out.print(snapshot.rightPwm);
    out.print(" kd=");
    out.println(snapshot.kd);
  }

  template <typename Out>
  static void printBluetoothTelemetry(Out& out,
                                      const TelemetrySnapshot& snapshot) {
    out.print("telem mode=");
    printMode(out, snapshot.mode);
    out.print(" angle=");
    out.print(snapshot.angleDegrees);
    out.print(" target=");
    out.print(snapshot.targetAngleDegrees);
    out.print(" auto=");
    out.print(snapshot.autoArmEnabled ? "on" : "off");
    out.print(" autoErr=");
    out.print(snapshot.autoAngleErrorDegrees);
    out.print(" gyroFresh=");
    out.print(snapshot.gyroFresh ? "yes" : "no");
    out.print(" gyroRate=");
    out.print(snapshot.gyroRateDegPerSec);
    out.print(" rate=");
    out.print(snapshot.filteredRateDegPerSec);
    out.print(" balance=");
    out.print(snapshot.balanceOutput);
    out.print(" left=");
    out.print(snapshot.leftMotor);
    out.print(" right=");
    out.println(snapshot.rightMotor);
  }

  template <typename Out>
  static void printRuntimeFields(Out& out,
                                 const RuntimeStatsSnapshot& runtime) {
    out.print(" loopTicks=");
    out.print(runtime.balanceTicks);
    out.print(" workUs=");
    out.print(runtime.lastWorkMicros);
    out.print(" maxWorkUs=");
    out.print(runtime.maxWorkMicros);
    out.print(" missed=");
    out.print(runtime.missedDeadlines);
    out.print(" feedbackFull=");
    out.print(runtime.fullFeedbackRefreshes);
    out.print(" feedbackLight=");
    out.print(runtime.lightFeedbackRefreshes);
    out.print(" motorWrites=");
    out.print(runtime.motorWrites);
    out.print(" motorStops=");
    out.print(runtime.motorStops);
    out.print(" telemUs=");
    out.print(runtime.lastTelemetryMicros);
    out.print(" maxTelemUs=");
    out.print(runtime.maxTelemetryMicros);
  }

private:
  template <typename Out>
  static void printResetCause(Out& out, uint8_t resetRaw) {
    const ResetDiagnostics::ResetCauseFlags flags =
        ResetDiagnostics::decode(resetRaw);
    bool hasPrinted = false;
    printResetFlag(out, flags.powerOn, "por", hasPrinted);
    printResetFlag(out, flags.external, "ext", hasPrinted);
    printResetFlag(out, flags.brownOut, "bor", hasPrinted);
    printResetFlag(out, flags.watchdog, "wdt", hasPrinted);
    printResetFlag(out, flags.jtag, "jtag", hasPrinted);
    printResetFlag(out, flags.unknown, "unknown", hasPrinted);
    if (!hasPrinted) {
      out.print("none");
    }
  }

  template <typename Out>
  static void printResetFlag(Out& out, bool present, const char* label,
                             bool& hasPrinted) {
    if (!present) {
      return;
    }
    if (hasPrinted) {
      out.print(",");
    }
    out.print(label);
    hasPrinted = true;
  }

  template <typename Out>
  static void printHexByte(Out& out, uint8_t value) {
    printHexNibble(out, static_cast<uint8_t>((value >> 4) & 0x0F));
    printHexNibble(out, static_cast<uint8_t>(value & 0x0F));
  }

  template <typename Out>
  static void printHexNibble(Out& out, uint8_t value) {
    const char digit =
        value < 10 ? static_cast<char>('0' + value)
                   : static_cast<char>('A' + (value - 10));
    out.print(digit);
  }

  template <typename Out>
  static void printCommonControlFields(Out& out,
                                       const TelemetrySnapshot& snapshot) {
    out.print(" angle=");
    out.print(snapshot.angleDegrees);
    out.print(" upright=");
    out.print(snapshot.uprightAngleDegrees);
    out.print(" trim=");
    out.print(snapshot.trimDegrees);
    out.print(" target=");
    out.print(snapshot.targetAngleDegrees);
    out.print(" bp=");
    out.print(snapshot.activeBalancePointDegrees);
    out.print(" auto=");
    out.print(snapshot.autoArmEnabled ? "on" : "off");
    out.print(" autoErr=");
    out.print(snapshot.autoAngleErrorDegrees);
    out.print(" gyroFresh=");
    out.print(snapshot.gyroFresh ? "yes" : "no");
    out.print(" gyroRate=");
    out.print(snapshot.gyroRateDegPerSec);
    out.print(" rate=");
    out.print(snapshot.filteredRateDegPerSec);
    out.print(" raw=");
    out.print(snapshot.rawBalanceOutput);
    out.print(" balance=");
    out.print(snapshot.balanceOutput);
    out.print(" speed=");
    out.print(snapshot.averageSpeedRpm);
    out.print(" pos=");
    out.print(snapshot.averagePositionDegrees);
    out.print(" hold=");
    out.print(snapshot.travelHoldTargetCorrectionDegrees);
    out.print(" left=");
    out.print(snapshot.leftMotor);
    out.print(" right=");
    out.print(snapshot.rightMotor);
    out.print(" kp=");
    out.print(snapshot.kp);
  }
};

#endif
