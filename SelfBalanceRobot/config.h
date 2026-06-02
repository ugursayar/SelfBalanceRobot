#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

enum class GyroAxis : uint8_t {
  X,
  Y,
  Z
};

namespace Config {
  constexpr uint8_t GyroPort = 0x06;
  constexpr GyroAxis BalanceGyroAxis = GyroAxis::X;

  constexpr uint8_t RightEncoderSlot = 1;
  constexpr uint8_t LeftEncoderSlot = 2;

  constexpr unsigned long BalanceLoopMicros = 10000UL;
  constexpr unsigned long CommandTimeoutMillis = 1500UL;
  constexpr unsigned long CalibrationMillis = 1200UL;

  constexpr float BalanceKp = 42.0f;
  constexpr float BalanceKi = 0.0f;
  constexpr float BalanceKd = 1.0f;
  constexpr float BalanceRateFilterAlpha = 0.75f;
  constexpr float WheelSpeedTargetCorrectionDegreesPerRpm = 0.0f;
  constexpr float MaxWheelSpeedTargetCorrectionDegrees = 0.0f;
  constexpr float WheelSpeedDampingCommandPerRpm = 0.0f;
  constexpr float WheelSpeedDampingMaxAngleErrorDegrees = 2.0f;
  constexpr float TravelHoldTargetDegreesPerWheelDegree = 0.002f;
  constexpr float MaxTravelHoldTargetCorrectionDegrees = 0.5f;
  constexpr float IntegralLimitDegreesSeconds = 30.0f;
  constexpr float MinBalanceBoostAngleDegrees = 0.80f;
  constexpr float LargeLeanBoostAngleDegrees = 2.0f;
  constexpr float LargeLeanBoostCommandPerDegree = 6.0f;

  constexpr float BalanceAngleTrimDegrees = -2.3f;
  constexpr float MinRuntimeKp = 0.0f;
  constexpr float MaxRuntimeKp = 200.0f;
  constexpr float MinRuntimeKi = 0.0f;
  constexpr float MaxRuntimeKi = 10.0f;
  constexpr float MinRuntimeKd = 0.0f;
  constexpr float MaxRuntimeKd = 20.0f;
  constexpr float MinRuntimeTrimDegrees = -5.0f;
  constexpr float MaxRuntimeTrimDegrees = 5.0f;
  constexpr float FallAngleDegrees = 35.0f;
  constexpr float StillAngleDeltaDegrees = 1.5f;
  constexpr float MaxStartupUprightAngleDegrees = 12.0f;
  constexpr bool EnableAutoArm = true;
  constexpr uint16_t BalancePointEepromAddress = 0;
  constexpr float AutoArmDefaultBalancePointDegrees = 0.70f;
  constexpr float MinPersistedBalancePointDegrees = -12.0f;
  constexpr float MaxPersistedBalancePointDegrees = 12.0f;
  constexpr float AutoArmAngleWindowDegrees = 3.0f;
  constexpr float AutoArmMaxRateDegPerSec = 12.0f;
  constexpr unsigned long AutoArmStillMillis = 900UL;
  constexpr unsigned long AutoArmStopCooldownMillis = 3000UL;

  constexpr unsigned long BalancePointLearningSettleMillis = 1500UL;
  constexpr unsigned long BalancePointLearningStableMillis = 1000UL;
  constexpr unsigned long BalancePointMinWriteIntervalMillis = 30000UL;
  constexpr float BalancePointLearningMaxAngleErrorDegrees = 1.0f;
  constexpr float BalancePointLearningMaxRateDegPerSec = 5.0f;
  constexpr int16_t BalancePointLearningMaxMotorCommand = 45;
  constexpr float BalancePointLearningAlpha = 0.25f;

  constexpr int16_t MotorDeadband = 0;
  constexpr int16_t MinBalanceMotorCommand = 16;
  constexpr int16_t MaxMotorCommand = 255;
  constexpr int16_t MotorTestCommand = 45;
  constexpr unsigned long MotorTestMillis = 700UL;

  constexpr bool EnableDebugSerial = true;
  // Intentionally enabled for cable-free test-control firmware.
  constexpr bool EnableBluetoothTestControl = true;
  constexpr unsigned long BluetoothBaud = 115200UL;
  constexpr unsigned long BluetoothTelemetryPeriodMillis = 250UL;
  constexpr unsigned long DebugPeriodMillis = 50UL;

  // Ramp the balance target from uprightAngle down to (upright+trim) over this
  // many milliseconds at the start of each balancing session.  Prevents the
  // large initial error spike that occurs when the robot is calibrated well
  // above the true balance point.
  constexpr unsigned long BalanceTargetRampMillis = 1500UL;
}

#endif
