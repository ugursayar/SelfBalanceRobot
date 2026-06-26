# SelfBalanceRobot

Arduino project for a MakeBlock MegaPi two-wheel self-balancing robot. The current firmware defaults to the diagnostic tuning profile: USB commands, Bluetooth telemetry/control, EEPROM balance-point commands, full motor authority, and wheel-speed target correction disabled while the angle loop is being tuned. EEPROM balance-point learning is available but starts off during unstable bring-up.

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

Start with the robot held securely. The sketch boots with motors disabled; balancing can start from USB/Bluetooth `ARM` or from cable-free auto-arm when the gyro is nearly still near the active balance point. The current diagnostic profile uses `Config::BareBalanceFirmware = false`; set it to `true` only when you intentionally want the stripped-down bare-balance build.

## Bring-Up

Read `docs/bring-up.md` before enabling the motors on the floor. Start with the robot held securely and use `STOP` immediately if correction direction is wrong.

## Performance Diagnostics

The diagnostic firmware (`BareBalanceFirmware = false`) includes runtime counters in `STATUS` for loop work time, missed balance-loop periods, encoder feedback refreshes, motor output writes, and telemetry print time. The bare profile compiles most of that runtime work out so the balance loop can run at 200 Hz with fewer serial and encoder side effects.

## Balance Controller (PID or LQR)

The balance control law is selectable at compile time, the same way the Bluetooth channel is gated. The default is a PID controller with a near-upright gain schedule; set `Config::EnableLqrController = true` in `SelfBalanceRobot/config.h` to run a Linear-Quadratic Regulator (state-feedback) law instead. Gains for both controllers live in `config.h`, and the active controller's diagnostics flow through the same `STATUS` fields.

For a stripped-down, encoder-driven pure-LQR build with no serial/Bluetooth/telemetry, see the standalone `LqrBalance/` sketch below.

## Next Hardware Validation

With the diagnostic profile uploaded, verify the corrected rate sign first: while tipping forward, `angle=` should move negative and `rate=` should also be negative while the forward tilt is increasing. Then tune from a held `ARM`. Cable-free auto-arm uses the stored absolute balance point, not the freshly calibrated hold angle; for the current hardware, `BP=0.00` is the starting cable-free test point. If cable-free auto-arm drives forward immediately, lower `BP`; if it drives backward immediately, raise `BP`.

## LqrBalance — standalone LQR sketch

[`LqrBalance/`](LqrBalance/) is a separate, stripped-down sketch that runs a pure full-state LQR (with encoder feedback) and nothing else — no serial, Bluetooth, telemetry, or runtime options. It is for testing the LQR control law in isolation on the same MegaPi hardware. See [`LqrBalance/README.md`](LqrBalance/README.md) for build, operation, and tuning, and [`LqrBalance/theory.md`](LqrBalance/theory.md) for deriving the gain matrix.

## Design

See the design spec in `docs/superpowers/specs/2026-05-16-makeblock-self-balancing-robot-design.md`.
