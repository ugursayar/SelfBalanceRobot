# Bluetooth Test Control Channel Design

## Purpose

Restore Bluetooth as a cable-free test and tuning console only. The immediate problem is that the robot can learn and persist an absolute balance point while the USB cable is pulling on the chassis, then fall forward when tested without the cable. Bluetooth must let us arm, stop, inspect, clear, set, and tune the robot without changing the balance algorithm or bringing back drive/ultrasonic behavior.

## Goals

- Accept the same newline command format from USB serial and Bluetooth serial.
- Use Bluetooth on the MegaPi hardware UARTs used by Makeblock firmware. The firmware listens on both `Serial2` and `Serial3` at `115200` baud, enabled by config.
- Keep Bluetooth scoped to testing and control.
- Add balance-point commands so a bad EEPROM value can be diagnosed and removed without re-uploading firmware.
- Add runtime switches for auto-arm and balance-point learning so tests can isolate the balance algorithm from persistence.
- Keep command parsing in native-testable C++ instead of growing `SelfBalanceRobot.ino`.
- Document a cable-free test sequence for future balancing work.

## Non-Goals

- Do not restore drive mode, obstacle avoidance, ultrasonic behavior, or joystick/app-specific commands.
- Do not change PID math, motor sign conventions, encoder feedback behavior, or balance-point learning thresholds in this feature.
- Do not make runtime tuning persist PID or trim values to EEPROM.

## Architecture

Extract the current inline USB command parser into a shared parser module. The sketch will read complete lines from USB `Serial` plus Bluetooth candidate ports `Serial2` and `Serial3`, feed them to the same parser, then apply the parsed action through the existing sketch-owned components.

Planned modules:

- `CommandParser`: pure line parser. It lowercases commands, validates argument counts and numeric ranges, and returns a parsed action.
- `CommandReader`: small `Stream` line reader that owns the per-port input buffer and invokes `CommandParser` when a newline arrives.
- `SelfBalanceRobot.ino`: remains responsible for applying parsed actions because it owns motors, PID state, auto-arm, learning, EEPROM store, and telemetry.

This keeps parsing unit-testable while avoiding a large ownership refactor of the sketch.

## Command Surface

Existing commands continue to work from both USB and Bluetooth:

| Command | Behavior |
|---|---|
| `ARM` | Start manual calibration/balancing through `RobotState`. |
| `STOP` | Stop motors, disarm, cancel motor test, and suppress auto-arm briefly. |
| `M+` | Brief motor test while disarmed. |
| `M-` | Brief motor test while disarmed. |
| `PID <kp> <ki> <kd>` | Clamp and apply runtime PID gains. |
| `TRIM <degrees>` | Clamp and apply runtime trim for manual calibration sessions. |

New test commands:

| Command | Behavior |
|---|---|
| `BP?` | Print active balance point, stored/default status, and EEPROM write count. |
| `BP SET <degrees>` | Save a persisted absolute balance point during tests, update auto-arm target, and acknowledge success or range rejection. |
| `BP <degrees>` | Short alias for `BP SET <degrees>`. |
| `BP CLEAR` | Invalidate stored balance-point records, fall back to default balance point, stop motors, and suppress auto-arm briefly. |
| `AUTO ON` / `AUTO OFF` | Temporarily enable or disable auto-arm until reset. |
| `LEARN ON` / `LEARN OFF` | Temporarily enable or disable balance-point learning until reset. |
| `STATUS` | Print one diagnostic snapshot on the requesting port. |
| `TELEM ON` / `TELEM OFF` | Enable or disable periodic debug telemetry on Bluetooth. USB debug behavior remains controlled by `EnableDebugSerial`. |

Unknown or malformed commands should print a short `error command` response and leave robot state unchanged.

## Safety

`STOP` has highest priority from either port. Any balance-point clear operation must also stop the motors and suppress auto-arm, because clearing a bad point while the robot is being handled should not immediately cause another arm attempt.

Balance-point learning starts enabled by default to preserve current behavior, but `LEARN OFF` blocks EEPROM writes from `BalancePointLearner` results. Auto-arm starts from `Config::EnableAutoArm`, but `AUTO OFF` blocks auto-arm even if the compile-time flag is enabled.

Motor test commands remain disarmed-only. PID, trim, and balance-point set commands are clamped by existing configuration ranges before taking effect.

The manual balance-point set command writes the same persisted absolute angle that auto-arm later uses. It is intended for controlled cable-free tests: find a candidate value, send `BP SET <degrees>`, then run auto-arm against that exact target.

## Telemetry

Command acknowledgements are written to the same port that issued the command. Important autonomous events, such as auto-arm and balance-point saves, continue to print to USB debug and should also print to Bluetooth when Bluetooth telemetry is enabled.

`STATUS` should include the key fields needed for cable-free balance diagnosis: mode, angle, target, trim, active balance point, persisted/default status, rate, raw output, final balance output, wheel speed, wheel position, hold correction, motor outputs, and PID gains.

## Cable-Free Test Plan

1. Upload firmware with USB, then disconnect the USB cable.
2. Connect over Bluetooth serial at `115200`.
3. Send `STOP`, `AUTO OFF`, `LEARN OFF`, and `BP CLEAR`.
4. Send `STATUS` and confirm `angle` changes when the robot tips forward/backward.
5. Use `ARM` with the robot held still, release only after the controller settles, and record `angle`, `target`, `balance`, `left`, `right`, and `rate`.
6. If the robot consistently falls forward, adjust the target using `TRIM` for manual sessions or `BP SET <degrees>` for auto-arm sessions, one small step at a time.
7. Once a cable-free target holds for short tests, send `BP SET <degrees>` to persist it.
8. Re-enable `AUTO ON` only after the persisted target is verified.
9. Re-enable `LEARN ON` only after the robot can balance without immediate divergence.

## Testing

Native tests should cover:

- Existing commands still parse case-insensitively.
- PID, trim, `BP SET <degrees>`, and `BP <degrees>` parse only with required arguments.
- Balance-point set rejects out-of-range values through the existing store range.
- `BP CLEAR`, `AUTO`, `LEARN`, `STATUS`, and `TELEM` commands parse correctly.
- Each `CommandReader` instance maintains its own partial line buffer so USB and Bluetooth input cannot corrupt each other.

Arduino compile remains the hardware integration check for the selected Bluetooth serial port and Makeblock includes.
