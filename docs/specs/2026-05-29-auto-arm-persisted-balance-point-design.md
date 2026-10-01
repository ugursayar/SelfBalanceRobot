# Auto-Arm Persisted Balance Point Design

Date: 2026-05-29

## Goal

Let the robot start cable-free by automatically entering balance mode when the gyro shows it is standing nearly still around a known balance point. Persist that balance point in EEPROM so later tests can reuse the best discovered angle without depending on a USB cable or serial `ARM`.

## Current Behavior

The sketch currently starts in `Disarmed`, waits for a serial `ARM`, samples the held angle during calibration, then balances around `uprightAngle + BalanceAngleTrimDegrees`. This worked for cable-connected testing, but the cable changes the physical balance point and makes free-standing tests hard to repeat.

## Chosen Approach

Persist the absolute gyro balance angle in EEPROM.

On boot, the firmware loads a saved absolute balance angle if the EEPROM record is valid. In `Disarmed`, it watches the gyro. If the live angle stays close to the persisted balance angle and the angular rate remains low for a configurable stillness window, the robot auto-arms and enters balancing around that persisted angle.

If EEPROM is empty or corrupt, the firmware uses a configured default balance angle and prints that no learned balance point was loaded. Manual `ARM` remains supported so a user can still force calibration and create a first learned point.

## Safety Behavior

Auto-arm is intentionally conservative:

- It runs only while `RobotState` is `Disarmed`.
- It requires fresh gyro data.
- It requires the live angle to remain within a configurable angle window around the persisted balance point.
- It requires the measured gyro rate to remain below a configurable stillness threshold.
- It requires those checks to hold continuously for a configurable duration.
- `STOP` immediately disarms and starts an auto-arm cooldown so the robot does not re-arm while the user is catching or repositioning it.
- Existing fall detection and fault handling remain unchanged.

The thresholds live in `config.h` so real-world tuning can happen without changing control logic.

## Learning Behavior

The firmware updates EEPROM only after a stable balancing window. A learned sample is accepted when:

- the robot has been balancing for at least a configurable settle time,
- the current angle is near the active target,
- the angular rate is low,
- the balance motor command is modest,
- enough time has passed since the previous EEPROM write.

When accepted, the new persisted value is a smoothed blend of the previous stored balance point and the newly observed balance point. This avoids writing noise into EEPROM and reduces wear.

Manual `ARM` can also produce a learned value after the same stable-balancing checks pass. This gives the robot a way to learn the first good cable-free balance point and then reuse it on later power cycles.

## EEPROM Format

Create a small EEPROM-backed module responsible for validation and persistence. The record stores:

- magic number,
- version,
- absolute balance angle in degrees,
- write counter,
- checksum.

The firmware ignores the record if magic, version, checksum, or angle range validation fails. This prevents random EEPROM contents from arming the robot around a nonsensical angle.

## Components

### BalancePointStore

New module that owns EEPROM record read/write. It exposes:

- `begin()` to load the record,
- `hasStoredBalancePoint()` to report validity,
- `balancePointDegrees()` to return the active stored value,
- `saveBalancePoint(float degrees)` to persist a validated learned value.

The module does not know about robot state or sensors.

### AutoArmController

New pure C++ module that watches sensor frames while disarmed and decides when auto-arm conditions have held long enough. It exposes:

- `configure(...)` for thresholds,
- `setTargetBalancePoint(float degrees)`,
- `reset()`,
- `suppressUntil(unsigned long nowMillis, unsigned long cooldownMillis)`,
- `update(const SensorFrame& frame)` returning whether auto-arm should trigger.

Keeping this logic separate makes it unit-testable without Arduino hardware.

### SelfBalanceRobot.ino Integration

The sketch loads the persisted balance point in `setup()`. During each loop:

1. Read serial commands.
2. Read sensors and motor feedback.
3. If disarmed and auto-arm is enabled, feed the frame into `AutoArmController`.
4. If auto-arm triggers, set the active balance target to the persisted balance point and enter balancing.
5. If manual `ARM` triggers, preserve the current calibration path.
6. While balancing, run the learning gate and persist a smoothed balance point only after stable behavior.

## Target Semantics

Auto-arm balancing uses the persisted absolute balance angle directly as the target. It does not apply `BalanceAngleTrimDegrees` on top, because the stored value already represents the real balance point.

Manual arming keeps the current calibration flow for compatibility. After it reaches stable balancing, the learned absolute target is persisted, so future auto-arm sessions can skip cable-dependent calibration.

## Serial Diagnostics

Debug serial output should make the behavior visible when a cable is attached:

- whether EEPROM loaded a valid balance point,
- when auto-arm candidate stillness begins or resets,
- when auto-arm enters balancing,
- when EEPROM is updated,
- when EEPROM is ignored as invalid.

These messages are diagnostic only; auto-arm must work without serial attached.

## Tests

Native tests should cover:

- EEPROM record accepts valid magic/version/checksum and rejects invalid records.
- Auto-arm does not trigger when angle is outside the window.
- Auto-arm does not trigger when gyro rate is above the stillness threshold.
- Auto-arm triggers only after the full stillness duration.
- `STOP` cooldown suppresses immediate re-auto-arm.
- Learning accepts stable balancing samples and rejects unstable samples.
- Existing manual arming, calibration, and fall behavior remain unchanged.

Arduino compile must still pass for `arduino:avr:mega`.

## Out Of Scope

This feature does not restore Bluetooth drive control or ultrasonic obstacle avoidance. It does not tune PID gains automatically. It does not write EEPROM continuously while unstable.
