#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <MeMegaPi.h>

namespace Config {
  constexpr uint8_t GyroPort = PORT_6;
  constexpr uint8_t UltrasonicPort = PORT_7;

  constexpr uint8_t RightMotorPort = PORT1B;
  constexpr uint8_t LeftMotorPort = PORT2B;

  constexpr bool InvertRightMotor = false;
  constexpr bool InvertLeftMotor = true;

  constexpr unsigned long BalanceLoopMicros = 10000UL;
  constexpr unsigned long UltrasonicPeriodMillis = 80UL;
  constexpr unsigned long CommandTimeoutMillis = 600UL;
  constexpr unsigned long CalibrationMillis = 1200UL;

  constexpr float BalanceKp = 18.0f;
  constexpr float BalanceKi = 0.0f;
  constexpr float BalanceKd = 0.8f;

  constexpr float MaxTargetLeanDegrees = 5.0f;
  constexpr float FallAngleDegrees = 35.0f;
  constexpr float StillAngleDeltaDegrees = 4.0f;
  constexpr float ObstacleStopDistanceCm = 25.0f;

  constexpr int16_t MotorDeadband = 8;
  constexpr int16_t MaxMotorCommand = 160;
  constexpr int16_t MaxTurnCommand = 50;
  constexpr int16_t MaxDriveCommand = 50;

  constexpr bool EnableRuntimeTuning = true;
  constexpr bool EnableDebugSerial = true;
  constexpr unsigned long DebugPeriodMillis = 200UL;
}

#endif
