# SelfBalanceRobot

Arduino project for a MakeBlock MegaPi two-wheel self-balancing robot with Bluetooth control and ultrasonic obstacle avoidance.

## Hardware Defaults

- MegaPi programmed as Arduino Mega 2560
- MegaPi Shield for RJ25
- Gyro sensor on RJ25 `PORT_6`
- Ultrasonic sensor on RJ25 `PORT_7`
- Right motor driver on MegaPi port 1
- Left motor driver on MegaPi port 2
- Standard MakeBlock Bluetooth adapter/controller

## Arduino Setup

1. Install the Arduino IDE.
2. Install the Makeblock Arduino library so `MeMegaPi.h` is available.
3. Open `SelfBalanceRobot/SelfBalanceRobot.ino`.
4. Select `Arduino Mega 2560 or Mega ADK`.
5. Select the MegaPi serial port.
6. Upload.

Start with the robot held securely. The sketch boots with motors disabled and requires an arm command before calibration and balancing.

## Bring-Up

Read `docs/bring-up.md` before enabling the motors on the floor. Start with the robot held securely and use `STOP` immediately if correction direction is wrong.

## Design

See the design spec in `docs/superpowers/specs/2026-05-16-makeblock-self-balancing-robot-design.md`.
