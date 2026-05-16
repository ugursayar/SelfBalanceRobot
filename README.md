# SelfBalanceRobot

Arduino project for a MakeBlock MegaPi two-wheel self-balancing robot with Bluetooth control and ultrasonic obstacle avoidance.

The current design targets:

- MegaPi programmed as Arduino Mega 2560
- MegaPi Shield for RJ25
- Gyro sensor on RJ25 `PORT_6`
- Ultrasonic sensor on RJ25 `PORT_7`
- Right motor driver on MegaPi port 1
- Left motor driver on MegaPi port 2
- Standard MakeBlock Bluetooth adapter/controller

See the design spec in `docs/superpowers/specs/2026-05-16-makeblock-self-balancing-robot-design.md`.
