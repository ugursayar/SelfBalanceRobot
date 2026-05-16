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
  constexpr uint8_t UltrasonicPort = 0x07;
  constexpr GyroAxis BalanceGyroAxis = GyroAxis::X;

  constexpr uint8_t RightMotorPort = 0x09;
  constexpr uint8_t LeftMotorPort = 0x0a;

  constexpr bool InvertRightMotor = false;
  constexpr bool InvertLeftMotor = true;
  constexpr bool InvertBalanceOutput = true;

  constexpr unsigned long BalanceLoopMicros = 10000UL;
  constexpr unsigned long UltrasonicPeriodMillis = 80UL;
  constexpr unsigned long CommandTimeoutMillis = 1500UL;
  constexpr unsigned long CalibrationMillis = 1200UL;

  constexpr float BalanceKp = 12.0f;
  constexpr float BalanceKi = 0.0f;
  constexpr float BalanceKd = 0.35f;

  constexpr float BalanceAngleTrimDegrees = 0.5f;
  constexpr float MaxTargetLeanDegrees = 3.0f;
  constexpr float FallAngleDegrees = 35.0f;
  constexpr float StillAngleDeltaDegrees = 4.0f;
  constexpr float MaxStartupUprightAngleDegrees = 12.0f;
  constexpr float ObstacleStopDistanceCm = 25.0f;

  constexpr float MinRuntimeKp = 0.0f;
  constexpr float MaxRuntimeKp = 100.0f;
  constexpr float MinRuntimeKi = 0.0f;
  constexpr float MaxRuntimeKi = 10.0f;
  constexpr float MinRuntimeKd = 0.0f;
  constexpr float MaxRuntimeKd = 20.0f;

  constexpr int16_t MotorDeadband = 5;
  constexpr int16_t MaxMotorCommand = 120;
  constexpr int16_t MaxTurnCommand = 30;
  constexpr int16_t MaxDriveCommand = 30;

  constexpr bool EnableRuntimeTuning = true;
  constexpr bool EnableDebugSerial = true;
  constexpr unsigned long DebugPeriodMillis = 200UL;
}

#endif
