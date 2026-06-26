# LqrBalance

A standalone, stripped-down **pure full-state LQR** balance controller for the
MakeBlock MegaPi two-wheel robot. No serial, no Bluetooth, no telemetry, no
runtime options — power on, stand it up near upright, and it balances.

This is an experimental sibling to the main firmware in
[`../SelfBalanceRobot`](../SelfBalanceRobot) (the full-featured build with
USB/Bluetooth, EEPROM, diagnostics, and a *selectable* PID-or-LQR controller).
`LqrBalance` exists to test the LQR control law in isolation, with the encoders
live and the smallest possible hot loop.

## Control law

Full-state feedback `u = K·x`, with `x = [tilt, tiltRate, wheelPos, wheelVel]`,
evaluated every 5 ms (200 Hz). Tilt and tilt-rate come from the gyro; wheel
position and velocity from the encoders. See [theory.md](theory.md) for the
linearized plant model and a runnable Python script that computes `K`.

## Hardware

Same as the main firmware: MegaPi programmed as Arduino Mega 2560, gyro on RJ25
`PORT_6`, right motor on slot 1, left motor on slot 2. See the
[top-level README](../README.md#hardware-defaults) for wiring.

## Build & upload

Requires the Makeblock Arduino library so `MeMegaPi.h` resolves (same setup as
the main firmware).

Arduino IDE: select board **Arduino Mega 2560 or Mega ADK**, open
`LqrBalance.ino`, choose the port, upload.

arduino-cli:

```
arduino-cli compile --fqbn arduino:avr:mega:cpu=atmega2560 LqrBalance
arduino-cli upload  -p <PORT> --fqbn arduino:avr:mega:cpu=atmega2560 LqrBalance
```

## Operation

- **Power on holding it at its balance point, still, for ~1–2 s** —
  `MeGyro.begin()` samples the gyro bias during that window (moving it then
  causes drift), and the firmware then **latches the tilt it reads as the balance
  reference** for this power-up. That reference centers the arm window + LED
  finder; `kBalancePointDeg` is only a fallback if the startup read is bad.
- **Arming LED:** while disarmed, the onboard LED blinks *faster* the closer the
  tilt is to the balance reference, and goes **solid when it engages**. The tilt
  you are holding at the instant it arms is captured as the balance setpoint.
- **Fall cutoff:** past `kFallAngleDeg` it disarms (motors off, LED resumes
  blinking) and re-arms automatically when stood back up.

## Tuning

Every knob is a named constant at the top of the sketch. They ship seeded from
the main firmware's tuning and **must be tuned on the bench** for your robot —
change one at a time, re-flash, observe.

| Constant | Role |
|---|---|
| `kK_tilt`, `kK_tiltRate` | tilt-angle and tilt-rate feedback gains — the core balance loop |
| `kK_wheelVel`, `kK_wheelPos` | wheel-speed damping and station-keeping — **bench-verified POSITIVE on this robot** (opposite to theory.md's sign); the position term tends to ring up (see CLAUDE.md) |
| `kSmallErrorDegrees`, `kSmallErrorGainScale` | near-upright softening of the tilt gain (1.0 = off / linear) |
| `kTiltRateFilterAlpha` | low-pass on the gyro rate before the damping term (0 = raw; tames the near-balance ± dither) |
| `kWheelVelFilterAlpha` | low-pass on the finite-differenced wheel velocity (0 = raw; tames encoder quantization noise) |
| `kCoastDeadbandPwm` | commands below this coast (0) instead of kicking — smooth settle through balance (0 = off) |
| `kMaxPwmStepPerTick`, `kMaxPwmStepFast` | magnitude-aware slew-rate limit — small commands ramp at `kMaxPwmStepPerTick` (smooths the stepping wobble: the big no-wobble win), large commands up to `kMaxPwmStepFast`. **Currently both 30 (uniform):** a fast big-command ramp rang up / drove away on hard pushes |
| `kWheelTermClampPwm` | bounds the combined wheel-term contribution — safety against a hot/wrong-signed wheel gain |
| `kMaxPwm` | output ceiling (out of 255 full duty) — caps recovery authority |
| `kPowerMin`, `kPowerFullDeg` | **curved power** — output scaled to `kPowerMin` of full for small corrections, ramping (quadratic) to full power by `kPowerFullDeg` of lean: gentle near balance, full authority for big leans. Lower `kPowerMin` at higher battery charge (see note) |
| `kBalancePointDeg` | fallback balance tilt; the live reference is latched from the tilt at power-up and the setpoint is captured at arm |
| `kArmWindowDeg`, `kArmMaxRateDegPerSec` | how close to upright / how steady before it arms |
| `kFallAngleDeg` | disarm threshold |

**Wheel-gain signs (bench-verified on this robot): both POSITIVE** in this code's
convention — *opposite* to theory.md's cart-pole sign, because the encoder/motor
wiring here is flipped from the model. Re-verify after any wiring change: while
balancing, spin a wheel by hand (should *resist*) and nudge the robot forward
(should *creep back*); the wheel-term clamp + fall cutoff keep a wrong sign
recoverable. **Station-keeping is the hard part on this robot:** the position
term (`kK_wheelPos`) tends to ring up, so the most stable runs have used velocity
damping with little or no position term (drive-away then becomes the ceiling).
See [CLAUDE.md](CLAUDE.md) and theory.md for the model-derived `K` path.

**Battery voltage is a hidden gain.** Motor torque per PWM count scales with pack
voltage, so the *effective* loop gain rises as the battery charges — the same
config feels calm on a near-empty pack and over-driven ("too strong") on a full
one. `kPowerMin` (the curve's small-correction power floor) is the manual
compensation: lower it at full charge, raise it as the pack drains. The proper
fix is to read pack voltage (e.g. the Waveshare UPS 3S over I²C, its own address
on the gyro's I²C bus) and scale `kPowerMin` by `Vnominal / Vmeasured` live, so
behavior is constant at any charge — not yet wired. Until then, tune at a known,
consistent charge level.

No telemetry ships in this sketch by design; tuning is by observation and the
LED. To find the exact balance-point angle, temporarily add a serial print of
the tilt, read it, then remove it.
