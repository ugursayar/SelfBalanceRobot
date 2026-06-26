/*
 * LqrBalance — pure full-state LQR balance controller for the MakeBlock MegaPi
 * (ATmega2560) two-wheel robot.  No serial, no Bluetooth, no telemetry, no
 * runtime options: power on, stand it up near upright, and it balances.
 *
 * Operation: while disarmed, the onboard LED blinks faster the closer the tilt
 * is to the balance point (a hands-free arming aid).  Hold it steady inside the
 * arm window and it captures that exact tilt as the balance setpoint and engages
 * (LED goes solid) — so the balance point re-sets itself to wherever you hold
 * it, with no fixed value to measure and no telemetry.  Tip past the fall angle
 * and it disarms (LED resumes blinking).  The near-upright tilt response is
 * softened so micro-corrections don't shake the frame.
 *
 * ---------------------------------------------------------------------------
 * Control law (full-state feedback, evaluated every tick):
 *
 *     u = K . (x - x*)
 *     x = [ tilt , tiltRate , wheelPos , wheelVel ]
 *
 * Sign convention: gains are in the motor-command convention, so a POSITIVE u
 * drives the wheels in the direction that recovers a FORWARD lean (matches the
 * MegaPi motor wiring used by the original firmware).  With that convention the
 * tilt gains are positive; the wheel (station-keeping / damping) gains come out
 * NEGATIVE — see the derivation by each constant below.
 *
 * K is meant to be computed offline (see theory.md): linearize the wheeled
 * inverted pendulum about upright, pick Q/R, solve the Riccati equation, and
 * paste the row here.  The defaults seed tilt/tiltRate from the known-good hand
 * tuning and use small, clamped wheel gains so the loop is stable and the
 * encoder states are live out of the box — but the wheel-gain SIGNS must be
 * verified on the bench (see "BENCH CHECK" notes).
 *
 * ---------------------------------------------------------------------------
 * Performance tricks applied (AVR-GCC, Arduino default -Os):
 *  - single translation unit, all state in file-scope globals: no heap, no
 *    virtuals, no dynamic dispatch, no String/Serial overhead.
 *  - strict 32-bit float math: every literal carries an 'f' suffix and every
 *    library value is cast to float so nothing silently promotes to double.
 *  - one micros() read per tick; integer loop-rate gate (no float time math).
 *  - wheel velocity is finite-differenced from the encoder position we already
 *    read, so MeEncoderOnBoard::updateSpeed() is never called.
 *  - the control law is a single fused multiply-add chain (branch-free).
 *  - identical motor PWM is not re-sent: redundant setMotorPwm() register
 *    writes are skipped.
 *  - hand-rolled float abs/clamp (no libcall); non-blocking LED (no delay()).
 *  - watchdog reset every loop() entry: a stall reboots the board.
 *
 * Build: Arduino IDE, board "MakeBlock MegaPi" (ATmega2560), with the MakeBlock
 * library installed (same environment as SelfBalanceRobot.ino).  For an extra
 * few percent, compile at -O2 via a platform.local.txt override.
 */

#include <MeMegaPi.h>
#include <avr/io.h>
#include <avr/wdt.h>

// Break any watchdog-reset loop the instant the chip boots, before setup()
// re-arms it.  (A watchdog reset leaves the WDT enabled; clearing it here keeps
// a transient stall from bricking the board into a reset cycle.)
void disableWatchdogOnBoot()
    __attribute__((naked)) __attribute__((section(".init3")));
void disableWatchdogOnBoot() {
  MCUSR = 0;
  wdt_disable();
}

// ===========================================================================
// Tuning constants  (all the knobs live here)
// ===========================================================================

// --- Hardware map (matches the original firmware) --------------------------
static const uint8_t kGyroI2cPort = 0x06;  // MeGyro (MPU6050) port
static const uint8_t kRightSlot = 1;        // MeEncoderOnBoard slot 1
static const uint8_t kLeftSlot = 2;         // MeEncoderOnBoard slot 2

// --- Loop timing -----------------------------------------------------------
static const uint32_t kLoopMicros = 5000UL;     // 200 Hz control loop

// --- Operating point -------------------------------------------------------
// Fallback balance tilt (deg).  On every reset the firmware latches the robot's
// actual tilt at power-up into gBalancePointDeg (see setup) and uses THAT as the
// arm-window + LED-finder center -- so you just hold it at its balance point
// while it boots.  This constant is only the default used if the startup gyro
// read is bad (NaN).
static const float kBalancePointDeg = 0.55f;

// --- LQR gain row K = [tilt, tiltRate, wheelPos, wheelVel] ------------------
// Tilt / tilt-rate: seeded from the proven hand tuning (strong, positive).
static const float kK_tilt = 16.00f;       // command per deg of tilt error
static const float kK_tiltRate = 1.50f;    // best run (hand-tuned)
// Wheel velocity (deg/s): damps wheel motion.  NEGATIVE so the command opposes
// the way the wheels are already turning (adds virtual friction -> stabilizing).
// BENCH CHECK: spin a wheel by hand while balancing; the bot should resist, not
// run away.  If it runs away, flip this sign.
static const float kK_wheelVel = 0.40f;    // best run; POSITIVE on this bot (velocity damping)
// Wheel position (deg from arm point): station-keeping.  NEGATIVE so a forward
// displacement commands a return toward the origin (cart-pole result: drive the
// base back, let it lean, catch it).  This is the touchiest gain.
// BENCH CHECK: nudge the balanced bot forward; it should creep back to where it
// started.  If it accelerates away, flip this sign.  Start near 0 and raise.
static const float kK_wheelPos = 0.0f;     // best run: station-keeping OFF (rings up on this bot; drive-away is the ceiling)

// Near-upright softening of the tilt (proportional) term.  LQR is linear, so
// without this it hits tiny tilt errors with the full kK_tilt and the resulting
// micro-corrections shake the frame.  Inside kSmallErrorDegrees the tilt gain is
// scaled toward kSmallErrorGainScale at the setpoint, ramping back to the full
// gain at the edge; beyond the band, full authority is preserved.
// Lower kSmallErrorGainScale (toward 0) = gentler small corrections.
static const float kSmallErrorDegrees = 2.5f;
static const float kSmallErrorGainScale = 1.0f;   // linear (softening traded against drive-away on this bot; reverted)

// Safety bound on the combined wheel (pos+vel) contribution, so an untuned /
// wrong-signed wheel gain can only ever add a recoverable amount before the
// fall-cutoff takes over.
static const float kWheelTermClampPwm = 255.0f; // effectively OFF (pure best run; wheelPos=0 keeps the wheel term small)

// Light low-pass on the finite-differenced wheel velocity to tame encoder
// quantization (0 = raw, ->1 = heavier smoothing).
static const float kWheelVelFilterAlpha = 0.0f;  // 0 = raw (pure LQR; expect quantization noise)

// Light low-pass on the gyro tilt rate before it feeds the kK_tiltRate (damping)
// term.  Without it, raw gyro noise rides into the rate term and dithers the
// command +/- near balance even at a steady lean; with it, damping is smooth.
// 0 = raw, ->1 = heavier smoothing (too high adds lag and erodes the damping).
static const float kTiltRateFilterAlpha = 0.0f;  // raw (validated-best/pure config; filter didn't change the feel)

// --- Output shaping --------------------------------------------------------
static const int16_t kMaxPwm = 255;        // motor command ceiling
// Master output scale ("power" dial).  Motor torque per PWM count scales with
// BATTERY VOLTAGE, so the effective loop gain RISES as the pack charges -- a
// config tuned on a near-empty battery is over-driven on a full one.  Lower this
// to take power out at full charge.  (Proper fix: battery-voltage compensation --
// set this to Vnominal/Vmeasured each loop once a battery-voltage reading exists.)
static const float kPowerScale = 0.45f;  // 0.45 = FIRST STABLE BALANCE (stays still) at ~FULL battery; raise as the pack drains, or use voltage comp
// Below this magnitude the motor is left to COAST (command 0) instead of being
// forced up to a minimum kick.  Forcing a floor here made the command snap to
// +/-min and flip sign as the robot settled through balance -- a reverse kick
// after every correction that felt like braking and set up a wobble.  Coasting
// lets it pass smoothly through zero.  Raise for a wider/calmer coast zone;
// lower toward 0 to keep driving small corrections (at the risk of the judder).
static const int16_t kCoastDeadbandPwm = 0;   // OFF for pure LQR (no coast zone)
// Slew-rate limit: the most the motor command may change per tick (200 Hz).
// Caps how fast the drive can ramp/reverse, so a correction can't slam from
// hard-drive to coast (or forward to reverse) in a single 5 ms step -- that
// one-tick jump is the "strong reflect then sudden brake" feel, and limiting it
// also attenuates high-frequency limit cycles.  Lower = smoother/gentler but
// slower recovery; set it >= kMaxPwm to disable.  At 20, the command ramps to
// full drive (100) in ~25 ms.
static const int16_t kMaxPwmStepPerTick = 512; // >= 2*kMaxPwm = OFF (no slew limit) for pure LQR

// --- Arm / safety ----------------------------------------------------------
static const float kArmWindowDeg = 4.0f;    // engage when within this of center
static const float kArmMaxRateDegPerSec = 30.0f;  // ...and roughly steady
static const float kFallAngleDeg = 35.0f;   // disengage (cut motors) past this
static const float kRateClampDegPerSec = 150.0f;  // reject absurd gyro spikes

// --- Arming LED (blinks faster as tilt nears the balance point) ------------
static const float kLedFarDeg = 20.0f;            // slowest blink at/over this
static const uint32_t kLedSlowMs = 500UL;         // far-from-point blink period
static const uint32_t kLedFastMs = 70UL;          // in-window blink period

// ===========================================================================
// Globals
// ===========================================================================
static MeGyro gGyro(kGyroI2cPort);
static MeEncoderOnBoard gRight(kRightSlot);
static MeEncoderOnBoard gLeft(kLeftSlot);

static uint32_t gLastTickMicros = 0;
static bool gArmed = false;
static float gBalanceSetpointDeg = kBalancePointDeg;  // captured at arm time
static float gBalancePointDeg = kBalancePointDeg;     // latched from tilt at reset
static float gPrevWheelPosDeg = 0.0f;
static float gWheelVelFiltDegPerSec = 0.0f;
static float gTiltRateFiltDegPerSec = 0.0f;  // low-passed gyro rate for damping
// Last command sent, for skip-if-unchanged.  Seeded to an impossible value so
// the first driveMotors() call always writes (guarantees an explicit stop).
static int16_t gLastPwm = INT16_MIN;
static int16_t gAppliedPwm = 0;  // slew-limited command being ramped toward target
// Non-blocking LED blink state.
static bool gLedOn = false;
static uint32_t gLedToggleMs = 0;

// ===========================================================================
// Small inline helpers (no libcalls)
// ===========================================================================
static inline float fabsFast(float v) { return v < 0.0f ? -v : v; }

static inline float clampF(float v, float limit) {
  if (v > limit) return limit;
  if (v < -limit) return -limit;
  return v;
}

// Tilt (proportional) gain after near-upright softening: full kK_tilt outside
// the band, scaled toward kSmallErrorGainScale at the setpoint.
static inline float scheduledTiltGain(float tiltErr) {
  if (kSmallErrorDegrees <= 0.0f) {
    return kK_tilt;
  }
  const float a = fabsFast(tiltErr);
  if (a >= kSmallErrorDegrees) {
    return kK_tilt;
  }
  const float ramp = a / kSmallErrorDegrees;  // 0 at setpoint, 1 at edge
  const float scale =
      kSmallErrorGainScale + ((1.0f - kSmallErrorGainScale) * ramp);
  return kK_tilt * scale;
}

// Replicates the MakeBlock Me_Megapi_encoder_direct timer setup required for
// slot 1 / slot 2 PWM to work.
static void configureMegaPiEncoderPwmTimers() {
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}

// Interrupt-driven quadrature decode (CHANGE edges, 2x resolution).  Direction
// rule matches the original firmware's handleRight/LeftPulse.
static void rightEncoderIsr() {
  const bool rising = (digitalRead(gRight.getPortA()) != 0);
  const bool portB = (digitalRead(gRight.getPortB()) != 0);
  if (rising ? portB : !portB) {
    gRight.pulsePosMinus();
  } else {
    gRight.pulsePosPlus();
  }
}

static void leftEncoderIsr() {
  const bool rising = (digitalRead(gLeft.getPortA()) != 0);
  const bool portB = (digitalRead(gLeft.getPortB()) != 0);
  if (rising ? portB : !portB) {
    gLeft.pulsePosMinus();
  } else {
    gLeft.pulsePosPlus();
  }
}

// Drive both wheels from a single balance command, skipping the write when the
// value is unchanged.  Wiring: right takes +u, left takes -u (mirrored mount).
static inline void driveMotors(int16_t u) {
  if (u == gLastPwm) {
    return;
  }
  gLastPwm = u;
  gRight.setMotorPwm(u);
  gLeft.setMotorPwm(static_cast<int16_t>(-u));
}

// Zero the wheel state so station-keeping holds the spot where we armed.
static inline void resetWheelState() {
  gRight.setPulsePos(0);
  gLeft.setPulsePos(0);
  gRight.updateCurPos();
  gLeft.updateCurPos();
  gPrevWheelPosDeg = 0.0f;
  gWheelVelFiltDegPerSec = 0.0f;
  gAppliedPwm = 0;  // start each balancing session ramping from zero drive
}

// Arming aid: solid LED while balancing; while disarmed, blink faster the closer
// the tilt is to the balance point so you can find the arm window by eye.
static void updateArmingLed(float armOffsetDeg, bool armed) {
  const uint32_t now = millis();
  if (armed) {
    if (!gLedOn) {
      gLedOn = true;
      digitalWrite(LED_BUILTIN, HIGH);
    }
    gLedToggleMs = now;
    return;
  }

  const float prox = fabsFast(armOffsetDeg);
  uint32_t period;
  if (prox <= kArmWindowDeg) {
    period = kLedFastMs;  // inside the window: fastest blink = ready to arm
  } else if (prox >= kLedFarDeg) {
    period = kLedSlowMs;
  } else {
    const float t = (prox - kArmWindowDeg) / (kLedFarDeg - kArmWindowDeg);
    period = kLedFastMs +
             static_cast<uint32_t>(t * static_cast<float>(kLedSlowMs - kLedFastMs));
  }

  if (now - gLedToggleMs >= period) {
    gLedToggleMs = now;
    gLedOn = !gLedOn;
    digitalWrite(LED_BUILTIN, gLedOn ? HIGH : LOW);
  }
}

// ===========================================================================
// Setup
// ===========================================================================
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  configureMegaPiEncoderPwmTimers();
  gGyro.begin();  // also brings up Wire / I2C and samples the gyro bias

  // Latch the orientation at this power-up as the balance point: let the
  // complementary filter settle (~300 ms), then mark the current tilt.  Hold the
  // robot at its balance point while it boots.  A bad (NaN) read keeps the
  // compiled default.  (The watchdog is still disabled here, so the delay safe.)
  for (uint8_t i = 0; i < 60; ++i) {
    gGyro.update();
    delay(5);
  }
  const float startTilt = static_cast<float>(gGyro.getAngleX());
  if (startTilt == startTilt) {  // not NaN
    gBalancePointDeg = startTilt;
  }

  attachInterrupt(gRight.getIntNum(), rightEncoderIsr, CHANGE);
  attachInterrupt(gLeft.getIntNum(), leftEncoderIsr, CHANGE);

  resetWheelState();
  driveMotors(0);

  gLastTickMicros = micros();
  // Re-arm the watchdog now that the slow gyro bring-up is done; the 200 Hz
  // loop pets it every ~5 ms, far inside the timeout, so only a real hang trips.
  wdt_enable(WDTO_250MS);
}

// ===========================================================================
// Control loop
// ===========================================================================
void loop() {
  wdt_reset();

  // Fixed-rate gate: one micros() read, integer compare, no float time math.
  const uint32_t nowMicros = micros();
  const uint32_t elapsedMicros = nowMicros - gLastTickMicros;
  if (elapsedMicros < kLoopMicros) {
    return;
  }
  gLastTickMicros = nowMicros;
  const float dtSeconds = static_cast<float>(elapsedMicros) * 0.000001f;
  const float invDt = 1.0f / dtSeconds;

  // ---- Read tilt state (fused angle + raw rate from the IMU) --------------
  gGyro.update();
  const float tilt = static_cast<float>(gGyro.getAngleX());
  // MeGyro integrates gyrY into angleX, so gyrY is the matching rate; the sign
  // matches the original firmware's BalanceGyroRateSign (-1).
  const float tiltRate =
      clampF(-static_cast<float>(gGyro.getGyroY()), kRateClampDegPerSec);
  // Smooth the rate before it feeds the damping term (raw gyro noise here is what
  // dithers the command +/- near balance).  Updated every tick so it never goes
  // stale; the arm-steadiness check below still uses the raw rate.
  gTiltRateFiltDegPerSec =
      (kTiltRateFilterAlpha * gTiltRateFiltDegPerSec) +
      ((1.0f - kTiltRateFilterAlpha) * tiltRate);

  // Offset from the expected balance point drives the arm window and LED finder.
  const float armOffset = tilt - gBalancePointDeg;
  updateArmingLed(armOffset, gArmed);

  // Bad IMU read -> motors off.  (tilt != tilt) catches NaN without math.h.
  if (tilt != tilt) {
    gArmed = false;
    driveMotors(0);
    return;
  }

  // ---- Arm gate -----------------------------------------------------------
  // Engage when held near the balance point AND roughly steady, and CAPTURE the
  // current tilt as the balance setpoint -- so the balance point re-sets itself
  // to wherever you hold it (no fixed value to measure, no telemetry).
  if (!gArmed) {
    driveMotors(0);
    if (fabsFast(armOffset) <= kArmWindowDeg &&
        fabsFast(tiltRate) <= kArmMaxRateDegPerSec) {
      gBalanceSetpointDeg = tilt;
      resetWheelState();
      gArmed = true;
    }
    return;
  }

  // ---- Control error about the captured setpoint; fall-cutoff -------------
  const float tiltErr = tilt - gBalanceSetpointDeg;
  if (fabsFast(tiltErr) > kFallAngleDeg) {
    gArmed = false;
    driveMotors(0);
    return;
  }

  // ---- Read wheel state (position now, velocity by finite difference) -----
  gRight.updateCurPos();
  gLeft.updateCurPos();
  const float wheelPosDeg =
      (static_cast<float>(-gRight.getCurPos()) +
       static_cast<float>(gLeft.getCurPos())) *
      0.5f;
  const float wheelVelRaw = (wheelPosDeg - gPrevWheelPosDeg) * invDt;
  gPrevWheelPosDeg = wheelPosDeg;
  gWheelVelFiltDegPerSec =
      (kWheelVelFilterAlpha * gWheelVelFiltDegPerSec) +
      ((1.0f - kWheelVelFilterAlpha) * wheelVelRaw);

  // ---- LQR state feedback (tilt term softened near upright) ---------------
  const float tiltCommand =
      (scheduledTiltGain(tiltErr) * tiltErr) +
      (kK_tiltRate * gTiltRateFiltDegPerSec);
  const float wheelCommand = clampF((kK_wheelPos * wheelPosDeg) +
                                        (kK_wheelVel * gWheelVelFiltDegPerSec),
                                    kWheelTermClampPwm);
  float u = tiltCommand + wheelCommand;
  u *= kPowerScale;  // master power dial: detune for full battery (placeholder for voltage comp)

  // ---- Output shaping: clamp -> slew-limit -> coast through tiny commands --
  u = clampF(u, static_cast<float>(kMaxPwm));
  const int16_t target = static_cast<int16_t>(u);

  // Slew-rate limit: step the applied command toward the target by at most
  // kMaxPwmStepPerTick, so the drive can't jump hard-drive -> coast or
  // forward -> reverse in one tick (the abrupt "braking" feel that builds wobble).
  int16_t step = static_cast<int16_t>(target - gAppliedPwm);
  if (step > kMaxPwmStepPerTick) {
    step = kMaxPwmStepPerTick;
  } else if (step < -kMaxPwmStepPerTick) {
    step = static_cast<int16_t>(-kMaxPwmStepPerTick);
  }
  gAppliedPwm = static_cast<int16_t>(gAppliedPwm + step);

  // Coast (de-energize) through tiny commands rather than humming at low PWM.
  int16_t command = gAppliedPwm;
  if (command > -kCoastDeadbandPwm && command < kCoastDeadbandPwm) {
    command = 0;
  }

  driveMotors(command);
}
