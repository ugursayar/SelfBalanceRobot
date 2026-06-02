# SelfBalanceRobot

Arduino project for a MakeBlock MegaPi two-wheel self-balancing robot. The current firmware is balance-only with USB serial commands and cable-free auto-arm from a persisted gyro balance point.

## Hardware Defaults

- MegaPi programmed as Arduino Mega 2560
- MegaPi Shield for RJ25
- Gyro sensor on RJ25 `PORT_6`
- Right motor driver on MegaPi port 1
- Left motor driver on MegaPi port 2

## Arduino Setup

1. Install the Arduino IDE.
2. Install the Makeblock Arduino library so `MeMegaPi.h` is available.
3. Open `SelfBalanceRobot/SelfBalanceRobot.ino`.
4. Select `Arduino Mega 2560 or Mega ADK`.
5. Select the MegaPi serial port.
6. Upload.

Start with the robot held securely. The sketch boots with motors disabled; balancing can start from a serial `ARM` or from cable-free auto-arm when the gyro is nearly still near the persisted balance point.

## Bring-Up

Read `docs/bring-up.md` before enabling the motors on the floor. Start with the robot held securely and use `STOP` immediately if correction direction is wrong.

## Performance Diagnostics

The firmware includes runtime counters in `STATUS` for loop work time, missed balance-loop periods, encoder feedback refreshes, motor output writes, and telemetry print time. Use these counters before changing `BalanceLoopMicros`, PID math, encoder sign conventions, or motor output behavior.

## Design

See the design spec in `docs/superpowers/specs/2026-05-16-makeblock-self-balancing-robot-design.md`.
