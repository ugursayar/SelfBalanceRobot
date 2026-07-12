# Bring-Up Guide

Use short tests and keep the robot held securely until motor direction and balance correction direction are confirmed.

## USB and Bluetooth Commands

Commands are case-insensitive and newline-terminated. USB Serial Monitor uses `Serial` at 115200 baud; set line ending to `Newline` or `Both NL & CR`. Bluetooth test control listens on MegaPi `Serial3` at 115200 baud when `EnableBluetoothTestControl` is true. `Serial2` is reserved for the RPi primary control link.

The default build currently has `Config::BareBalanceFirmware = false`. In this diagnostic profile, USB commands, Bluetooth test control, Bluetooth telemetry, and EEPROM balance-point commands are enabled. EEPROM balance-point learning is available but starts off during unstable bring-up. Target-angle wheel-speed correction and direct wheel-speed damping are disabled while the inner angle loop is being tuned. Automatic USB debug streaming is off by default so serial printing does not disturb the balance loop; use `STATUS` snapshots on USB instead. Set `BareBalanceFirmware = true` in `SelfBalanceRobot/config.h` only when you intentionally want the stripped-down bare-balance firmware.

| Command | Description |
|---|---|
| `ARM` | Calibrate upright angle; enter balancing when done. |
| `STOP` | Stop motors immediately, return to disarmed, and suppress auto-arm briefly. |
| `M+` | Brief motor test: both wheels drive backward. Robot must be disarmed. |
| `M-` | Brief motor test: both wheels drive forward. Robot must be disarmed. |
| `PID <kp> <ki> <kd>` | Update balance gains live. Example: `pid 24 0 0.8` |
| `TRIM <degrees>` | Apply a live/manual relative target adjustment for manual calibration sessions. Example: `trim -0.5` |
| `BP?` | Print current balance point, stored/default status, and EEPROM write count. |
| `BP SET <degrees>` | Persist an absolute gyro balance-point angle for auto-arm tests while stopped/disarmed. Example: `bp set 0.00` |
| `BP <degrees>` | Short alias for `BP SET <degrees>`. |
| `BP CLEAR` | Clear learned EEPROM balance point, fall back to default, stop motors, and suppress auto-arm briefly. |
| `AUTO ON` / `AUTO OFF` | Enable or disable auto-arm until reset. |
| `LEARN ON` / `LEARN OFF` | Enable or disable balance-point EEPROM learning until reset. |
| `STATUS` | Print one diagnostic snapshot. |
| `TELEM ON` / `TELEM OFF` | Enable or disable periodic shorter Bluetooth-only telemetry at a slower period than USB debug output. |

`PID` and `TRIM` take effect on the next balance loop tick and reply on the command channel.
`BP SET` is accepted only while fully disarmed: no motor test, no active calibration/balancing/fault, no fresh `ARM`, and no `STOP`/`BP CLEAR` cooldown.
Do not copy a `TRIM` offset into `BP SET`; persist the absolute target/balance-point angle reported by diagnostics.
`STATUS` includes `reset=` and `resetRaw=`. Reset tokens are `por` (power-on), `ext` (reset pin), `bor` (brownout), `wdt` (watchdog), and `jtag`. If live PID settings return to the compiled default after motors wake, check these fields.

`STATUS` also reports runtime loop counters:

- `workUs`: work time for the most recent balance tick.
- `maxWorkUs`: highest observed balance tick work time since boot.
- `missed`: count of balance ticks whose interval exceeded `BalanceLoopMicros`.
- `feedbackFull` / `feedbackLight`: encoder feedback refresh counts.
- `motorWrites` / `motorStops`: hardware motor output calls.
- `telemUs` / `maxTelemUs`: latest and peak telemetry formatting time.

When tuning performance in the diagnostic firmware, collect one `STATUS` snapshot with Bluetooth telemetry off and one with Bluetooth telemetry on. Prefer changes that reduce `maxWorkUs` and keep `missed` stable at zero before considering a faster balance loop.

### Performance Validation Pass

This pass applies to the diagnostic firmware (`BareBalanceFirmware = false`). After uploading firmware, keep the robot held securely and collect `STATUS` snapshots in this order:

1. Booted and disarmed with Bluetooth telemetry off.
2. Disarmed after sending `TELEM ON`.
3. During a short held `ARM`/balancing session.
4. Immediately after `STOP`.

Use those snapshots before changing control behavior:

- `missed=0` means the balance loop is keeping its period.
- `maxWorkUs` should stay well below `BalanceLoopMicros`.
- `feedbackFull` should increase periodically, and every balance tick only when a speed-based correction or Bluetooth telemetry explicitly requires full feedback.
- `motorWrites` should not climb every tick when the commanded output is steady.
- `motorStops` should not climb continuously while the robot is already idle.

If these checks pass, prioritize mechanical balance, trim, PID, and travel-hold tuning. If `missed` rises or `maxWorkUs` approaches `BalanceLoopMicros`, reduce telemetry or loop work before considering a faster loop rate.

Auto-arm is enabled by default but uses a tight diagnostic gate: the robot must be close to the active balance point and nearly still before motors enable. In diagnostic mode, EEPROM can provide a learned balance point; if EEPROM is empty, the firmware uses `AutoArmDefaultBalancePointDegrees`. The current hardware's starting cable-free test point is stored as `BP=0.00`; `BP=0.70` was observed to command forward drive during auto-arm. Use `AUTO OFF` while manually tuning if the robot re-arms too aggressively after a catch. `STOP` still disarms immediately and starts a short auto-arm cooldown.

## First Checks

1. Upload with wheels off the ground.
2. Open Serial Monitor at 115200 baud.
3. Confirm debug output shows a changing `angle=` value when you tilt the robot.
4. Run `m-` and confirm both wheels spin forward. Run `m+` and confirm both spin backward.
5. Hold the robot upright and still, then send `ARM`.
6. Wait for the LED to become solid (fast blink = calibrating, slow blink = fault).
7. Tilt the robot gently and confirm wheels correct in the direction that would drive under the falling body — forward tilt should produce forward wheel motion.
8. If the balance angle does not change when tipping forward/backward, change `BalanceGyroAxis` in `config.h`.
9. Confirm debug output includes `speed=`, `left=`, and `right=` values that change when the wheels rotate.

## Release Technique

**This is the most important factor for successful balance.**

1. Hold the robot with both hands, as upright and as still as possible.
2. Send `ARM`. Keep holding — the LED fast-blinks during calibration (~1.2 s).
3. When the LED turns solid, the robot is balancing. **Do not release immediately.**
4. Wait 500 ms–1 s with the LED solid. Watch the `angle=` value in debug output — let the motors settle the angle toward the target (`target=` field) before releasing.
5. **Peel fingers away slowly** — do not drop or jerk. The robot should be within ~1–2° of `target=` before you fully let go.
6. If `angle=` is still swinging far from `target=` when the LED turns solid, send `STOP`, reposition more carefully, and try again.

The first angular rate sample after release is the best predictor of outcome:
- `rate=` < 5°/s → likely stable
- `rate=` > 10°/s → will diverge; catch and retry

## Cable-Free Auto-Arm

1. Power the robot without the USB cable attached.
2. Hold it near the known balance point and as still as possible. For the current hardware, start around the stored `BP=0.00` point.
3. Wait for auto-arm to enter balancing; the LED becomes solid when balancing starts.
4. Keep holding for another 500 ms-1 s so the controller settles before release.
5. If it re-arms too aggressively after a catch, send `STOP` while connected or power-cycle and increase `AutoArmStopCooldownMillis`.

In diagnostic mode, the cable-free balance point is stored in EEPROM as an absolute gyro angle. USB/Bluetooth `ARM` calibrates the current held angle, but cable-free auto-arm uses the stored `BP` target instead. If cable-free auto-arm immediately drives forward, the stored `BP` is too high; lower it in small steps such as `bp set -0.30` or `bp set -0.50`. If it immediately drives backward, the stored `BP` is too low; raise it in small steps. Balance-point learning starts off by default; turn `LEARN ON` back on only after the robot can balance without immediate divergence.

## Bluetooth Cable-Free Test Flow

This flow uses the default diagnostic profile (`BareBalanceFirmware = false`).

1. Upload over USB, then disconnect the USB cable.
2. Connect to the Bluetooth serial module at 115200 baud. The firmware listens on `Serial3`.
3. Send `STOP`, `AUTO OFF`, `LEARN OFF`, and `BP SET 0.00`.
4. Send `STATUS` and confirm `angle=` changes when tipping forward/backward.
5. For manual tests, hold the robot still and send `ARM`.
6. If it falls forward immediately, stop, adjust the manual target in small relative steps with `TRIM <degrees>`, and retry.
7. For cable-free auto-arm, tune the stored absolute `BP` directly. If auto-arm drives forward immediately, lower `BP`; if it drives backward immediately, raise `BP`.
8. Turn `LEARN ON` back on only after the robot can balance without immediate divergence.

## Balance Tuning

### Current Diagnostic Baseline (config.h as of 2026-06-04)

```
BareBalanceFirmware = false
BalanceLoopMicros = 10000
BalanceKp = 24.0
BalanceKi = 0.0
BalanceKd = 0.8
BalanceRateFilterAlpha = 0.75
BalanceGyroRateSign = -1.0
BalanceAngleTrimDegrees = 0.0       (start with no trim bias for inner-loop tests)
MinBalanceBoostAngleDegrees = 0.80
MinBalanceMotorCommand = 16
MaxMotorCommand = 255
LargeLeanBoostAngleDegrees = 2.0
LargeLeanBoostCommandPerDegree = 6.0
WheelSpeedTargetCorrectionDegreesPerRpm = 0.0
MaxWheelSpeedTargetCorrectionDegrees = 0.0
WheelSpeedDampingCommandPerRpm = 0.0
TravelHoldTargetDegreesPerWheelDegree = 0.0     (disabled during baseline tuning)
MaxTravelHoldTargetCorrectionDegrees = 0.5
StillAngleDeltaDegrees = 1.5
FallAngleDegrees = 35.0
SafetyCutoffAngleErrorDegrees = 0.0
SafetyCutoffMotorCommand = 0
SafetyCutoffMillis = 0
AutoArmDefaultBalancePointDegrees = 0.70
Stored hardware BP = 0.00                 (current cable-free test point)
AutoArmAngleWindowDegrees = 1.0
AutoArmMaxRateDegPerSec = 4.0
AutoArmStillMillis = 900
AutoArmStopCooldownMillis = 3000
EnableDebugSerial = false
EnableBluetoothTestControl = true
EnableBalancePointLearning = true
EnableBalancePointLearningByDefault = false
EnableMotorFeedback = true
```

Stop reason indicators:
- Rapid blink while disarmed: safety cutoff stopped the motors (only when safety cutoff is enabled).
- Slow blink in fault: fall, gyro, or calibration fault stopped balancing.
- LED off while disarmed: command/manual stop or normal idle.
- USB `STATUS` includes `stopReason=` when connected.

Sign conventions confirmed:
- `m-` drives forward, `m+` drives backward.
- Forward tilt makes `angle=` go negative.
- While forward tilt is increasing, `rate=` should also go negative. If it has the opposite sign, change `BalanceGyroRateSign`.
- Both wheels drive in the same robot direction.
- `BalanceAngleTrimDegrees = 0.0` keeps the manual `ARM` target at the calibrated hold angle. Add trim only after the inner angle loop stops launching or oscillating.

### Tuning Steps

1. Keep `BalanceKi` at `0.0` throughout initial tuning.
2. If the robot shakes quickly near upright, reduce `BalanceKp` or lower `MinBalanceMotorCommand`.
3. If the robot falls without correcting strongly enough, increase `BalanceKp` in small steps.
4. If `left=` and `right=` are near `MaxMotorCommand` but the robot still cannot recover, increase `MaxMotorCommand`.
5. Increase `BalanceKd` only enough to damp oscillation without making corrections jerky. At Kd > 1.5 with angular rates > 40°/s, the D-term can override the P-term and produce wrong-direction corrections.
6. After the robot can enter balancing without a launch, use `TRIM` to correct a consistent standing lean. Send small steps: `trim 0.2`, `trim -0.2`, `trim 0.4`, `trim -0.4`. Once you find the right value, copy it into `BalanceAngleTrimDegrees` in `config.h`.
7. Add a very small `BalanceKi` only after P, D, motor limits, and trim are stable.

You can tune gains live without re-uploading:

```
pid 22 0 0.7
pid 24 0 0.8
pid 28 0 0.6
pid 30 0 0.9
```

### Auto-Trim via PowerShell

If the calibration angle varies between tests (common when holding the robot at slightly different positions), use this PowerShell snippet to auto-apply a trim that targets a specific balance angle immediately after ARM:

```powershell
# Usage: run BEFORE sending ARM. Set $targetAngle to your desired balance angle.
$targetAngle = 0.00   # degrees — current cable-free starting point

$port = New-Object System.IO.Ports.SerialPort("COM8", 115200)
$port.Open()
$port.WriteLine("arm")

while ($true) {
    $line = $port.ReadLine()
    Write-Host $line
    if ($line -match 'mode-change=balancing upright=([-\d.]+)') {
        $upright = [float]$Matches[1]
        $trim = [math]::Round($targetAngle - $upright, 2)
        $port.WriteLine("trim $trim")
        Write-Host ">> injected trim=$trim (upright=$upright, target=$targetAngle)"
    }
}
```

### Diagnostic Hints

| Symptom | Likely cause | Action |
|---|---|---|
| `balance=` changes but `left=`/`right=` stay at zero | Deadband too high | Keep `MotorDeadband = 0` |
| `balance=` non-zero but motors don't respond until large lean | `MinBalanceMotorCommand` too low | Raise `MinBalanceMotorCommand` |
| Small corrections are jumpy or oscillate sign | D-term or min-boost too aggressive | Lower `BalanceKd`, lower `MinBalanceMotorCommand`, or raise `MinBalanceBoostAngleDegrees` |
| Robot drifts steadily in one direction | CG offset from upright setpoint | Use `TRIM` to find correct lean, then set `BalanceAngleTrimDegrees` |
| Cable-free auto-arm drives forward immediately | Stored `BP` is too high for the cable-free balance point | Lower stored `BP`, starting from `bp set 0.00` toward `bp set -0.30` |
| Cable-free auto-arm drives backward immediately | Stored `BP` is too low for the cable-free balance point | Raise stored `BP` in small steps |
| Robot drives away while angle stays near target | Angle loop is balancing the body but travel feedback is absent or too weak | Finish inner angle-loop tuning first, then re-enable speed correction only after encoder speed sign is confirmed |
| Robot slowly builds forward/backward oscillation | Travel hold too aggressive | Keep `TravelHoldTargetDegreesPerWheelDegree = 0.0` until speed correction is stable |
| Calibration faults with robot held still | `StillAngleDeltaDegrees` too tight | Raise to `1.5` (current default) |
| LED slow-blinks immediately on ARM | Calibration fault — robot moving too much, or startup angle > 12° | Hold robot more upright and still; check Serial for `mode-change=fault` |
| Board shows its two-blink reboot pattern after motors stop | Motor current spike, brownout, or battery/contact interruption | Check `STATUS reset=` while USB remains connected; keep `MaxMotorCommand` capped during diagnosis and secure the battery contacts |
| D-term causes wrong-direction correction | Rate sign is wrong or angular rate is too large with high Kd | Check `rate=` sign first; then reduce `BalanceKd` if needed |
| Speed damping makes forward travel worse | Direct damping was re-enabled with the wrong encoder sign or damping sign | Keep `WheelSpeedDampingCommandPerRpm = 0.0` during diagnostic tuning |

### Notes on Disabled Features

**Travel Hold** (`TravelHoldTargetDegreesPerWheelDegree`): Currently disabled for baseline tuning after wireless tests showed target drift during longer runs. Re-enable only after the robot is stable without it.

**Auto Arm Window** (`AutoArmAngleWindowDegrees`): Set to 1.0 degree in diagnostic mode, with a 4 deg/s rate gate. This prevents cable-free auto-arm from enabling while the robot is already leaning forward or backward. Balance-point learning starts off; send `LEARN ON` only after the robot can balance without immediate divergence.

**Wheel Speed Damping** (`WheelSpeedDampingCommandPerRpm`): Disabled at `0.0` in the diagnostic profile. Reintroduce direct damping only after the angle loop is stable and encoder speed signs are confirmed.

### Notes on Experimental Features

**Wheel Speed Target Correction** (`WheelSpeedTargetCorrectionDegreesPerRpm`): Disabled at `0.0` with a `0.0` degree cap during cable-connected wobble debugging. Re-enable it only after the robot can hold the inner angle target without fast oscillation; while this term is enabled, encoder speed is refreshed every balance tick so the velocity outer loop is not working from stale speed data.
