# Bring-Up Guide

Use short tests and keep the robot held securely until motor direction and balance correction direction are confirmed.

## USB and Bluetooth Commands

Commands are case-insensitive and newline-terminated. USB Serial Monitor uses `Serial` at 115200 baud; set line ending to `Newline` or `Both NL & CR`. Bluetooth test control uses `Serial1` at 115200 baud when `EnableBluetoothTestControl` is true.

| Command | Description |
|---|---|
| `ARM` | Calibrate upright angle; enter balancing when done. |
| `STOP` | Stop motors immediately, return to disarmed, and suppress auto-arm briefly. |
| `M+` | Brief motor test: both wheels drive backward. Robot must be disarmed. |
| `M-` | Brief motor test: both wheels drive forward. Robot must be disarmed. |
| `PID <kp> <ki> <kd>` | Update balance gains live. Example: `pid 42 0 0.18` |
| `TRIM <degrees>` | Apply a live/manual relative target adjustment for manual calibration sessions. Example: `trim -0.5` |
| `BP?` | Print current balance point, stored/default status, and EEPROM write count. |
| `BP SET <degrees>` | Persist an absolute gyro balance-point angle for auto-arm tests while stopped/disarmed. Example: `bp set 0.85` |
| `BP <degrees>` | Short alias for `BP SET <degrees>`. |
| `BP CLEAR` | Clear learned EEPROM balance point, fall back to default, stop motors, and suppress auto-arm briefly. |
| `AUTO ON` / `AUTO OFF` | Enable or disable auto-arm until reset. |
| `LEARN ON` / `LEARN OFF` | Enable or disable balance-point EEPROM learning until reset. |
| `STATUS` | Print one diagnostic snapshot. |
| `TELEM ON` / `TELEM OFF` | Enable or disable periodic shorter Bluetooth-only telemetry at a slower period than USB debug output. |

`PID` and `TRIM` take effect on the next balance loop tick and reply on the command channel.
`BP SET` is rejected while motors are enabled; send it only while stopped/disarmed.
Do not copy a `TRIM` offset into `BP SET`; persist the absolute target/balance-point angle reported by diagnostics.

Auto-arm is enabled by default. If EEPROM contains a valid learned balance point, or if the configured default balance point is close enough for the current build, the robot can enter balancing without a serial `ARM` when it is held nearly still near that angle. `STOP` still disarms immediately and starts a short auto-arm cooldown.

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
2. Hold it near the known balance point and as still as possible.
3. Wait for auto-arm to enter balancing; the LED becomes solid when balancing starts.
4. Keep holding for another 500 ms-1 s so the controller settles before release.
5. If it re-arms too aggressively after a catch, send `STOP` while connected or power-cycle and increase `AutoArmStopCooldownMillis`.

The learned balance point is stored in EEPROM as an absolute gyro angle. During stable balancing, firmware may update it slowly using `BalancePointLearningAlpha`, but it will not write continuously while the robot is unstable.

## Bluetooth Cable-Free Test Flow

1. Upload over USB, then disconnect the USB cable.
2. Connect to the Bluetooth serial module at 115200 baud.
3. Send `STOP`, `AUTO OFF`, `LEARN OFF`, and `BP CLEAR`.
4. Send `STATUS` and confirm `angle=` changes when tipping forward/backward.
5. For manual tests, hold the robot still and send `ARM`.
6. If it falls forward immediately, stop, adjust the manual target in small relative steps with `TRIM <degrees>`, and retry.
7. Once a cable-free balance point works for short manual tests, note the absolute target/balance-point angle from diagnostics. Stop/disarm, send `BP SET <degrees>` with that absolute angle, then test `AUTO ON`.
8. Turn `LEARN ON` back on only after the robot can balance without immediate divergence.

## Balance Tuning

### Current Baseline (config.h as of 2026-05-29)

```
BalanceKp = 42.0
BalanceKi = 0.0
BalanceKd = 1.0
BalanceRateFilterAlpha = 0.75
BalanceAngleTrimDegrees = -2.3      (negative = lean forward; adjust live with TRIM)
MinBalanceBoostAngleDegrees = 0.80
MinBalanceMotorCommand = 16
LargeLeanBoostAngleDegrees = 2.0
LargeLeanBoostCommandPerDegree = 6.0
WheelSpeedTargetCorrectionDegreesPerRpm = 0.0   (disabled — sign not yet verified)
MaxWheelSpeedTargetCorrectionDegrees = 0.0
WheelSpeedDampingCommandPerRpm = 0.0            (disabled — sign issue, see note)
TravelHoldTargetDegreesPerWheelDegree = 0.002   (experimental)
MaxTravelHoldTargetCorrectionDegrees = 0.5
StillAngleDeltaDegrees = 1.5
AutoArmDefaultBalancePointDegrees = 0.70
AutoArmAngleWindowDegrees = 3.0
AutoArmMaxRateDegPerSec = 6.0
AutoArmStillMillis = 900
AutoArmStopCooldownMillis = 3000
BalancePointLearningAlpha = 0.25
```

Sign conventions confirmed:
- `m-` drives forward, `m+` drives backward.
- Forward tilt makes `angle=` go negative.
- Both wheels drive in the same robot direction.
- `BalanceAngleTrimDegrees = -2.3` compensates for the robot's CG being ~2.3° forward of the calibrated upright angle.

### Tuning Steps

1. Keep `BalanceKi` at `0.0` throughout initial tuning.
2. If the robot shakes quickly near upright, reduce `BalanceKp` or lower `MinBalanceMotorCommand`.
3. If the robot falls without correcting strongly enough, increase `BalanceKp` in small steps.
4. If `left=` and `right=` are near `MaxMotorCommand` but the robot still cannot recover, increase `MaxMotorCommand`.
5. Increase `BalanceKd` only enough to damp oscillation without making corrections jerky. At Kd > 1.5 with angular rates > 40°/s, the D-term can override the P-term and produce wrong-direction corrections.
6. Use `TRIM` to correct a consistent standing lean **before** changing gains. Send small steps: `trim 0.3`, `trim -0.3`, `trim 0.6`, `trim -0.6`. Once you find the right value, copy it into `BalanceAngleTrimDegrees` in `config.h`.
7. Add a very small `BalanceKi` only after P, D, motor limits, and trim are stable.

You can tune gains live without re-uploading:

```
pid 25 0 0.7
pid 35 0 0.9
pid 45 0 1.1
pid 60 0 1.4
```

### Auto-Trim via PowerShell

If the calibration angle varies between tests (common when holding the robot at slightly different positions), use this PowerShell snippet to auto-apply a trim that targets a specific balance angle immediately after ARM:

```powershell
# Usage: run BEFORE sending ARM. Set $targetAngle to your desired balance angle.
$targetAngle = 0.70   # degrees — the CG sweet spot, typically 0.5–1.0

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
| Robot slowly builds forward/backward oscillation | Travel hold or speed correction enabled | Reduce `TravelHoldTargetDegreesPerWheelDegree` toward `0.0` and confirm `WheelSpeedTargetCorrectionDegreesPerRpm = 0.0` |
| Calibration faults with robot held still | `StillAngleDeltaDegrees` too tight | Raise to `1.5` (current default) |
| LED slow-blinks immediately on ARM | Calibration fault — robot moving too much, or startup angle > 12° | Hold robot more upright and still; check Serial for `mode-change=fault` |
| D-term causes wrong-direction correction | Angular rate > 40°/s with high Kd | Reduce `BalanceKd`; keep Kd ≤ 1.0 until Kp is confirmed stable |
| `WheelSpeedDampingCommandPerRpm` enabled | Formula amplifies instead of damps in same-direction lean+spin | Keep at 0.0 until sign is re-verified with encoder direction confirmed |

### Notes on Disabled Features

**Travel Hold** (`TravelHoldTargetDegreesPerWheelDegree`): Currently enabled at a small experimental value. If forward/backward oscillation grows, reduce it toward `0.0` before changing PID gains.

**Wheel Speed Damping** (`WheelSpeedDampingCommandPerRpm`): Formula `balance -= speed * coeff` has the wrong sign when lean and wheel spin are in the same direction (both forward → amplifies forward drive). Keep at 0.0 until encoder sign conventions are confirmed and the formula is corrected.

**Wheel Speed Target Correction** (`WheelSpeedTargetCorrectionDegreesPerRpm`): Not yet tested with this hardware configuration. Keep at 0.0 until balance is fully stable without it.
