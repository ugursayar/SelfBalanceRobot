# LqrBalance — implementation notes

Standalone pure-LQR sketch for the MegaPi. The point of this sketch is the
minimal, dependency-light hot loop — **keep it serial-free**; don't add
telemetry or a command channel to the production path (a temporary serial print
for calibration is fine, but remove it).

## Hardware traps

- **PWM timer setup is mandatory.** `configureMegaPiEncoderPwmTimers()`
  (TCCR1/TCCR2) must run before driving slots 1/2 or the motors won't PWM.
  Copied verbatim from the Makeblock `Me_Megapi_encoder_direct` example.
- **`setMotorPwm(0)` coasts, it does not brake.** In
  `MakeBlockDrive/src/MeEncoderOnBoard.cpp` it writes `analogWrite(PWM, 0)` (0%
  duty → motor de-energized). It also has its own previous-value skip and a 5 µs
  direction-change deadtime. Don't expect PWM 0 to lock the wheels.
- **Motor / encoder signs are wiring-specific.** Drive is `right = +u`,
  `left = -u`; wheel position is `-right + left`. These mirror the main
  firmware's `Motors.cpp` — verify on hardware before changing either.
- **Encoder quadrature** uses CHANGE-edge ISRs with the
  `rising ? portB : !portB` rule (2× resolution), same as the main firmware.
- **Two Makeblock libraries are installed** (`MakeBlockDrive` and
  `MakeBlock_Drive_Updated`), both defining `MeEncoderOnBoard` / `MeMegaPi`.
  arduino-cli resolves to one; be aware if behavior changes between them.

## Control / behavior decisions

- **Balance point: latched at reset, then captured at arm.** `setup()` settles
  the gyro ~300 ms and latches the startup tilt into `gBalancePointDeg` (so hold
  the robot at its balance point through boot); that centers the arm window + LED
  finder. `kBalancePointDeg` is only the NaN fallback. The setpoint the controller
  actually holds is captured again at the instant it arms (`gBalanceSetpointDeg`).
- **Wheel-gain signs are bench-verified POSITIVE on this robot** — *opposite* to
  theory.md's negative cart-pole derivation (the encoder/motor convention here is
  flipped from the model), so a model-derived `K` needs its wheel-gain signs
  flipped to positive. **Station-keeping does not survive this robot's backlash:**
  the position term (`kK_wheelPos`) rings up at every magnitude tried — even
  +0.01, and even the model's coherent `+0.45` set — so the stable configs run
  velocity damping with `kK_wheelPos = 0` and accept slow drive-away. Treat
  station-keeping as a mechanical (backlash) problem, not a gain one. The
  `kWheelTermClampPwm` clamp + fall cutoff bound a wrong/hot wheel gain.
- **Battery voltage scales the effective gain (big one), handled by curved power.**
  Motor torque per PWM count is proportional to pack voltage, with no
  compensation, so the *effective* loop gain rises as the battery charges — a
  config tuned near-empty is over-driven when full. This was the hidden variable
  behind much of the session's "too strong"/inconsistent behavior. The fix is
  **curved power**: `kPowerMin` is the small-correction power fraction (the calm
  near-balance behavior), ramping quadratically to full power by `kPowerFullDeg`
  of lean — small corrections gentle, big leans full authority. `kPowerMin` still
  scales with pack voltage; the eventual fix is `kPowerMin = base ·
  Vnominal/Vmeasured` from a live read (Waveshare UPS 3S over I²C). Until wired,
  tune at a consistent charge level.
- **Coast deadband, not a min-PWM floor.** Forcing small commands up to a
  minimum made the command flip sign and reverse-kick as the robot settled
  through balance (felt like braking, set up a wobble). Coasting small commands
  fixes it — do not reintroduce a floor.
- **Input filters tame noise, not lag.** Raw gyro rate fed straight into the
  `kK_tiltRate` term dithers the command ± near balance (a steady lean gives a
  steady proportional term; only the noisy rate flips the sign), so
  `kTiltRateFilterAlpha` low-passes it. The finite-differenced wheel velocity gets
  `kWheelVelFilterAlpha` for the same reason (encoder quantization).
- **Slew-rate limiter (`kMaxPwmStepPerTick`).** The motor is driven open-loop —
  `setMotorPwm` is a direct register write each tick, there is **no internal motor
  "session"/PID** (we never call `runSpeed`/`move`/`loop`). So an abrupt
  hard-drive→coast or +→− jump in one 5 ms tick is felt as "braking" and pumps a
  limit cycle; the slew limit caps per-tick command change (this is the big
  no-wobble win). `gAppliedPwm` resets to 0 at each arm. It's **magnitude-aware**:
  `kMaxPwmStepPerTick` for small commands, ramping to `kMaxPwmStepFast` for large.
  But a fast big-command ramp **rings up / drives away on hard pushes** (backlash),
  so both are currently set equal (30, uniform) — firm hard-push recovery vs.
  calm is a hard trade on this drivetrain, and calm won.
- **All shaping is neutralizable to bare `u = K·x`.** Setting the gain schedule
  to 1.0, deadbands/alphas to 0, and slew/clamp high runs a pure linear LQR; the
  best balancing runs so far have been at or near that pure config.
- **Watchdog ordering.** Disabled in `.init3` (breaks a WDT-reset boot loop),
  re-enabled `WDTO_250MS` only *after* the slow `gGyro.begin()` in `setup()`, and
  petted at the top of every `loop()`. Don't move `wdt_enable` before the gyro
  bring-up or a normal boot can reset-loop.
- **Strict float.** Every literal carries an `f` suffix and library `double`s are
  cast to `float`; keep it that way (AVR `double` math is software-emulated).

## Testing

There is no native unit test for this sketch — it's hardware-coupled and is only
mock-compiled for syntax. The main firmware's selectable `LqrController` class is
unit-tested in [`../tests/native/test_lqr_controller.cpp`](../tests/native/test_lqr_controller.cpp).
